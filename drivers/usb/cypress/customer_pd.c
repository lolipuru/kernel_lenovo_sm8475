/*
 * Cypress USB Type-C and PD Controller Driver
 *
 * Author: Dudley Du <dudl@cypress.com>
 *
 * Copyright (C) 2016 Cypress Semiconductor, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include "cyccg.h"
#include "customer_pd.h"

#define PDPORT_NAME(port_name)	((port_name) ?: "NULL")
#define pd_err(fmt, port_name, ...)	pr_err("%s: %s: error: " fmt,	\
	__func__, PDPORT_NAME(port_name), ##__VA_ARGS__)
#define pd_warn(fmt, port_name, ...)	pr_warn("%s: %s: warning: " fmt, \
	__func__, PDPORT_NAME(port_name), ##__VA_ARGS__)
#define pd_info(fmt, port_name, ...)	pr_info("%s: %s: info: " fmt, \
	__func__, PDPORT_NAME(port_name), ##__VA_ARGS__)
#if defined(CONFIG_CYCCG_DEBUG) && CONFIG_CYCCG_DEBUG
#define pd_dbg(fmt, port_name, ...)	pr_err("%s: %s: dbg: " fmt,	\
	__func__, PDPORT_NAME(port_name), ##__VA_ARGS__)
#if defined(CONFIG_CYCCG_VDEBUG) && CONFIG_CYCCG_VDEBUG
#define pd_vdbg(fmt, port_name, ...)	pr_err("%s: %s: vdbg: " fmt,	\
	__func__, PDPORT_NAME(port_name), ##__VA_ARGS__)
#else
#define pd_vdbg(fmt, port_name, ...)
#endif  /* CONFIG_CYCCG_VDEBUG */
#else
#define pd_dbg(fmt, port_name, ...)
#define pd_vdbg(fmt, port_name, ...)
#endif  /* CONFIG_CYCCG_DEBUG */

struct usbc_pdport {
	struct list_head node;
	struct usbc_pdport_name_addr *name_addr;
	struct usbc_pdport_operations *ops;

	struct delayed_work delayed_work;
	spinlock_t slock;
	enum usbc_pdport_power_adapter_state pa_state;
	bool send_vdm_sync;
	struct completion done;
	int voltage_mV;
	int current_mA;

	/*
	 * TODO (Customer):
	 * Define any variables if required.
	 */
};

static LIST_HEAD(usbc_pdports_list);
static DEFINE_SPINLOCK(usbc_pdports_lock);

static void usbc_pdport_event_callback(char *port_name,
		u32 event_code, u8 *event_data, size_t size);
static void usbc_pdport_send_random_data_worker(struct work_struct *work);

static inline int usbc_pdport_init(struct usbc_pdport *pdport, char *name)
{
	struct usbc_pdport_operations *ops = NULL;
	struct usbc_pdport_name_addr *name_addr;
	int err;

	if (!pdport || !name)
		return -EINVAL;

	spin_lock_init(&pdport->slock);
	spin_lock(&pdport->slock);
	if (pdport->ops) {
		spin_unlock(&pdport->slock);
		return 0;	/* The PD port has been registered. */
	}
	spin_unlock(&pdport->slock);

	name_addr = cyccg_get_usbc_pdport_name_addr_by_name(name);
	if (!name_addr) {
		pd_warn("port_name=%s not found\n", name, name);
		return -ENODEV;
	}

	err = cyccg_usbc_pdport_event_register(name, &ops,
				usbc_pdport_event_callback);
	if (err || !ops || !ops->data_role_swap ||
			!ops->get_port_status || !ops->send_vdm) {
		pd_warn("failed to register event callback for port=%s, %d\n",
			name, name, err);
		return err;
	}

	spin_lock(&pdport->slock);
	pdport->name_addr = name_addr;
	pdport->ops = ops;
	spin_unlock(&pdport->slock);

	spin_lock(&usbc_pdports_lock);
	list_add(&pdport->node, &usbc_pdports_list);
	spin_unlock(&usbc_pdports_lock);

	INIT_DELAYED_WORK(&pdport->delayed_work,
			  usbc_pdport_send_random_data_worker);

	pd_vdbg("pdport initialized\n", name);

	return 0;
}

static inline void usbc_pdport_deinit(struct usbc_pdport *pdport)
{
	char *port_name;

	if (!pdport) {
		pd_err("invalid pdport=NULL\n", port_name);
		return;
	}

	cancel_delayed_work(&pdport->delayed_work);

	spin_lock(&pdport->slock);
	if (!pdport->name_addr) {
		spin_unlock(&pdport->slock);
		return;
	}

	port_name = pdport->name_addr->name;
	pdport->name_addr = NULL;
	pdport->ops = NULL;
	spin_unlock(&pdport->slock);

	cyccg_usbc_pdport_event_unregister(port_name);

	spin_lock(&usbc_pdports_lock);
	list_del_init(&pdport->node);
	spin_unlock(&usbc_pdports_lock);
}

static inline struct usbc_pdport_operations *usbc_pdport_get_ops(
		struct usbc_pdport *pdport)
{
	struct usbc_pdport_operations *ops;

	if (!pdport)
		return NULL;

	spin_lock(&pdport->slock);
	ops = pdport->ops;
	spin_unlock(&pdport->slock);
	return ops;
}

static inline char *usbc_pdport_get_name(struct usbc_pdport *pdport)
{
	char *port_name = NULL;

	if (!pdport)
		return NULL;

	spin_lock(&pdport->slock);
	if (pdport->name_addr)
		port_name = pdport->name_addr->name;
	spin_unlock(&pdport->slock);
	return port_name;
}


static struct usbc_pdport *usbc_pdport_name_to_pdport(char *port_name)
{
	struct usbc_pdport_name_addr *name_addr =
		cyccg_get_usbc_pdport_name_addr_by_name(port_name);
	struct usbc_pdport *pdport;

	if (!name_addr)
		return NULL;

	spin_lock(&usbc_pdports_lock);

	list_for_each_entry(pdport, &usbc_pdports_list, node) {
		if (pdport && pdport->name_addr == name_addr)
			goto out;
	}

	pdport = NULL;
out:
	spin_unlock(&usbc_pdports_lock);
	return pdport;
}

static inline enum usbc_pdport_power_adapter_state
usbc_pdport_get_pa_state(struct usbc_pdport *pdport)
{
	enum usbc_pdport_power_adapter_state pa_state;

	spin_lock(&pdport->slock);
	pa_state = pdport->pa_state;
	spin_unlock(&pdport->slock);
	return pa_state;
}

static inline void usbc_pdport_set_pa_state(struct usbc_pdport *pdport,
		enum usbc_pdport_power_adapter_state pa_state)
{
	spin_lock(&pdport->slock);
	pdport->pa_state = pa_state;
	spin_unlock(&pdport->slock);
}

static inline u8 pa_vdm_mA_to_current_level(int current_mA)
{
	return current_mA / PA_VDM_CURRENT_LEVEL_GRANULARITY;
}

static inline int pa_vdm_current_level_to_mA(u8 current_level)
{
	return current_level * PA_VDM_CURRENT_LEVEL_GRANULARITY;
}

static inline char *pa_vdm_current_level_to_A_string(u8 current_level)
{
	static char str[8];
	int current_mA = pa_vdm_current_level_to_mA(current_level);

	memset(str, 0, sizeof(str));
	scnprintf(str, sizeof(str), "%d.%dA",
		current_mA / 1000, (current_mA % 1000) / 100);
	return str;
}

static inline u8 pa_vdm_mV_to_voltage_value(int voltage_mV)
{
	return (voltage_mV / 1000) * 10 + (voltage_mV % 1000) / 100;
}

static inline int pa_vdm_voltage_value_to_mV(u16 voltage)
{
	return voltage * 100;
}

static inline char *pa_vdm_voltage_value_to_V_string(u16 voltage)
{
	static char str[8];
	int voltage_mA = pa_vdm_voltage_value_to_mV(voltage);

	memset(str, 0, sizeof(str));
	scnprintf(str, sizeof(str), "%d.%dV",
		voltage_mA / 1000, (voltage_mA % 1000) / 100);
	return str;
}

static inline u32 pa_vdm_generate_random_data(void)
{
	/*
	 * TODO (Customer):
	 * Generat and return real random number data.
	 */
	return 0x00000001;
}

static inline bool pa_vdm_is_valid_random_data(char *port_name, u32 random_data)
{
	u32 expected_random_data = 0x00000002;

	/*
	 * TODO (Customer):
	 * Verify the real random number data based on the algorithem to
	 * generate the random number data.
	 */

	pd_vdbg("received encripted random data=0x%08x, expected=0x%08x\n",
		port_name, random_data, expected_random_data);
	if (random_data == expected_random_data)
		return true;
	return false;
}

static bool usbc_pdport_is_ec_dfp_mode(struct usbc_pdport *pdport)
{
	struct usbc_pdport_operations *pdport_ops = usbc_pdport_get_ops(pdport);
	char *port_name = usbc_pdport_get_name(pdport);
	struct usbc_pdport_status port_status;
	int err;

	if (!pdport_ops || !port_name) {
		pd_err("invalid PD port instance\n", port_name);
		return false;
	}

	err = pdport_ops->get_port_status(port_name, &port_status);
	if (err) {
		pd_err("failed to read PD Port status, %d\n", port_name, err);
		return false;
	}

	if (!port_status.type_c_connected ||
			!port_status.contract_established) {
		pd_vdbg("type-c or PD not established\n", port_name);
		return false;
	}

	if (!port_status.current_data_role) {
		pd_vdbg("not in DFP mode\n", port_name);
		return false;
	}

	return true;
}

static void usbc_pdport_pd_vdm_cmd_error_process(struct usbc_pdport *pdport)
{
	if (usbc_pdport_get_pa_state(pdport) > PDPORT_PA_POWER_TUNE_READY)
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_POWER_TUNE_READY);
	else
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
}

static void usbc_pdport_pa_dr_swap_ready_process(struct usbc_pdport *pdport)
{
	char *port_name = usbc_pdport_get_name(pdport);
	bool dr_swapped = false;
	int err;

	pd_vdbg("<<<< enter\n", port_name);
	if (!port_name) {
		pd_err("invalid PD port instance\n", port_name);
		return;
	}

	switch (usbc_pdport_get_pa_state(pdport)) {
	case PDPORT_PA_DR_SWAPPED:
		pd_dbg("PDPORT_PA_DR_SWAPPED\n", port_name);
		if (!usbc_pdport_is_ec_dfp_mode(pdport)) {
			pd_err("data role swapped to DFP mode failed\n",
				port_name);
			usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
			break;
		}

#if 1	/* Wait Power Adapter get ready, remove it if FW not need it. */
		msleep(20);
#endif

		/*
		 * Now, the power and current tune VDM command can be sent to
		 * the Power Adapter device.
		 */
		dr_swapped = true;
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_POWER_TUNE_READY);

		/* fall-through */
	case PDPORT_PA_POWER_TUNE_READY:
		pd_dbg("PDPORT_PA_POWER_TUNE_READY\n", port_name);
		/*
		 * PDPORT_PA_POWER_TUNE_READY only can be changed after:
		 * 1) After DR_SWAP success event received;
		 * 2) Tune power current/voltage command sent.
		 * 3) Tune power current/voltage command encountered error.
		 *    At this situation, this case should not be re-entered.
		 */
		if (!dr_swapped) {
			pd_dbg("not expected VDM data, bypass\n", port_name);
			break;
		}

		/*
		 * TODO (Customer):
		 * If other threads/modules need to be notified to start
		 * send power current turm VDM command. In this case,
		 * usbc_pdport_tune_power_current_sync() interface in suggested
		 * to be used.
		 */

		/*
		 * TODO (Customer):
		 * Following in just an exapmple on how to send and receive
		 * the power and current tune command to the Power Adapter.
		 *
		 * Start to turn the power and current in same thread.
		 * When send failed, keep in ready state, wait for next power
		 * and current tune commnad.
		 */
		err = usbc_pdport_tune_power_current(port_name, 5000, 2000);
		if (err) {
			pd_err("failed to power current tune cmd, %d\n",
				port_name, err);
			break;
		}
		pd_dbg("send power tune voltate=%s, current=%s\b", port_name,
			pa_vdm_voltage_value_to_V_string(
				pa_vdm_mV_to_voltage_value(pdport->voltage_mV)),
			pa_vdm_current_level_to_A_string(
				pa_vdm_mA_to_current_level(
					pdport->current_mA)));
		break;
	default:
		break;
	}

	pd_vdbg(">>>> exit\n", port_name);
}

int usbc_pdport_tune_power_current(char *port_name,
		u16 power_mV, u16 current_mA)
{
	struct usbc_pdport *pdport = usbc_pdport_name_to_pdport(port_name);
	struct usbc_pdport_operations *pdport_ops = usbc_pdport_get_ops(pdport);
	union pa_vdm_current_voltage_data power_current;
	u8 vdm_data[USBPD_VDM_DATA_SIZE];
	int err;

	pd_vdbg("<<<< enter, power_mV=%u, current_mA=%u\n",
		port_name, power_mV, current_mA);
	if (!pdport_ops) {
		pd_err("invalid PD port instance, pdport_ops=0x%p\n",
			port_name, pdport_ops);
		return -EINVAL;
	}

	spin_lock(&pdport->slock);
	if (pdport->pa_state != PDPORT_PA_POWER_TUNE_READY) {
		if (pdport->pa_state > PDPORT_PA_POWER_TUNE_READY)
			err = -EBUSY;
		else
			err = -EINVAL;
		spin_unlock(&pdport->slock);
		pd_err("invalid power adapter state=%d, %d\n",
			port_name, (int)pdport->pa_state, err);
		return err;
	}
	pdport->pa_state = PDPORT_PA_POWER_TUNE_CMD_SENT;
	spin_unlock(&pdport->slock);

	pdport->send_vdm_sync = false;
	pdport->voltage_mV = power_mV;
	pdport->current_mA = current_mA;
	put_unaligned_le32(PA_VDM_OPCODE_POWER_TUNE_CMD, &vdm_data[0]);
	power_current.pa_request.voltage = pa_vdm_mV_to_voltage_value(power_mV);
	power_current.pa_request.current_level =
				pa_vdm_mA_to_current_level(current_mA);
	put_unaligned_le32(power_current.vdo,
				&vdm_data[USBPD_DATA_OBJ_SIZE]);
	err = pdport_ops->send_vdm(port_name, vdm_data,
				USBPD_DATA_OBJ_SIZE * 2);
	if (err) {
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_POWER_TUNE_READY);
		pd_err("failed to send VDM power tune cmd, %d\n",
			port_name, err);
		return err;
	}

	pd_vdbg(">>>> exit\n", port_name);
	return 0;
}

int usbc_pdport_tune_power_current_sync(char *port_name,
		u16 voltage_mV, u16 current_mA)
{
	struct usbc_pdport *pdport = usbc_pdport_name_to_pdport(port_name);
	struct usbc_pdport_operations *pdport_ops = usbc_pdport_get_ops(pdport);
	union pa_vdm_current_voltage_data power_current;
	u8 vdm_data[USBPD_VDM_DATA_SIZE];
	unsigned long timeout = 500;	/* units: ms */
	int err;

	pd_vdbg("<<<< enter, power_mV=%u, current_mA=%u\n",
		port_name, voltage_mV, current_mA);
	if (!pdport_ops) {
		pd_err("invalid PD port instance, pdport_ops=0x%p\n",
			port_name, pdport_ops);
		return -EINVAL;
	}

	spin_lock(&pdport->slock);
	if (pdport->pa_state != PDPORT_PA_POWER_TUNE_READY) {
		if (pdport->pa_state > PDPORT_PA_POWER_TUNE_READY)
			err = -EBUSY;
		else
			err = -EINVAL;
		spin_unlock(&pdport->slock);
		pd_err("invalid power adapter state=%d, %d\n",
			port_name, (int)pdport->pa_state, err);
		return err;
	}
	pdport->pa_state = PDPORT_PA_POWER_TUNE_CMD_SENT;
	spin_unlock(&pdport->slock);

	init_completion(&pdport->done);
	pdport->send_vdm_sync = true;
	pdport->voltage_mV = voltage_mV;
	pdport->current_mA = current_mA;
	put_unaligned_le32(PA_VDM_OPCODE_POWER_TUNE_CMD, &vdm_data[0]);
	power_current.pa_request.voltage =
				pa_vdm_mV_to_voltage_value(voltage_mV);
	power_current.pa_request.current_level =
				pa_vdm_mA_to_current_level(current_mA);
	put_unaligned_le32(power_current.vdo,
				&vdm_data[USBPD_DATA_OBJ_SIZE]);
	err = pdport_ops->send_vdm(port_name, vdm_data,
				USBPD_DATA_OBJ_SIZE * 2);
	if (err) {
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_POWER_TUNE_READY);
		pd_err("failed to send VDM power tune cmd, %d\n",
			port_name, err);
		return err;
	}

	timeout = wait_for_completion_timeout(&pdport->done,
						msecs_to_jiffies(timeout));
	if (timeout == 0) {
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_POWER_TUNE_READY);
		pd_err("timeout for wait VDM power tune cmd resp\n",
			port_name);
		return -ETIMEDOUT;
	}

	if (get_unaligned_le16(&power_current.pa_response.id) ==
			PA_VDM_CMD_ID) {
		pd_dbg("success done, latest power voltate=%s, current=%s\n",
			port_name, pa_vdm_voltage_value_to_V_string(
					power_current.pa_response.voltage),
			pa_vdm_current_level_to_A_string(
				power_current.pa_response.current_level));
	} else {
		pd_dbg("invalid CMD ID value received=0x%04x, expected=0x%04x",
			port_name,
			get_unaligned_le16(&power_current.pa_response.id),
			PA_VDM_CMD_ID);
	}

	usbc_pdport_set_pa_state(pdport, PDPORT_PA_POWER_TUNE_READY);
	pd_vdbg(">>>> exit\n", port_name);
	return 0;
}

static int usbc_pdport_send_random_data(struct usbc_pdport *pdport)
{
	struct usbc_pdport_operations *pdport_ops = usbc_pdport_get_ops(pdport);
	char *port_name = usbc_pdport_get_name(pdport);
	u8 vdm_data[USBPD_VDM_DATA_SIZE];
	int err;

	pd_vdbg("<<<< enter\n", port_name);
#if 1	/* Wait Power Adapter get ready, remove it if FW not need it. */
	msleep(20);
#endif

	put_unaligned_le32(PA_VDM_OPCODE_RANDOM_DATA, &vdm_data[0]);
	put_unaligned_le32(pa_vdm_generate_random_data(),
				&vdm_data[USBPD_DATA_OBJ_SIZE]);
	err = pdport_ops->send_vdm(port_name,
			vdm_data, USBPD_DATA_OBJ_SIZE * 2);
	if (err) {
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
		pd_warn("failed to send random data, %d\n", port_name, err);
		return err;
	}

	usbc_pdport_set_pa_state(pdport, PDPORT_PA_EC_RANDON_DATA_SENT);
	pd_vdbg(">>>> exit, success\n", port_name);
	return 0;
}

static void usbc_pdport_send_random_data_worker(struct work_struct *work)
{
	struct delayed_work *dwork =
			container_of(work, struct delayed_work, work);
	struct usbc_pdport *pdport =
			container_of(dwork, struct usbc_pdport, delayed_work);

	usbc_pdport_send_random_data(pdport);
}

static void usbc_pdport_pa_vdm_event_process(struct usbc_pdport *pdport,
		u8 *data, size_t size)
{
	struct usbc_pdport_operations *pdport_ops = usbc_pdport_get_ops(pdport);
	char *port_name = usbc_pdport_get_name(pdport);
	struct usbpd_vdm_message *vdm_msg = (struct usbpd_vdm_message *)data;
	enum vdm_sop_type sop_type = (enum vdm_sop_type)vdm_msg->sop_type;
	u32 pa_vdm_opcode = get_unaligned_le32(&vdm_msg->vdo[0]);
	union pa_vdm_current_voltage_data power_current;
	enum usbc_pdport_power_adapter_state pa_status;
	union vdm_msg_header msg_header;
	int err = 0;

	if (!pdport_ops || !port_name) {
		pd_err("invalid PD port instance\n", port_name);
		return;
	}

	msg_header.msg_header = le16_to_cpu(vdm_msg->msg_header);
	pd_vdbg("received VMD data, sop_type=%d size=%zu, data_objs=%u\n",
		port_name, sop_type, size, msg_header.data_objs);
	pd_vdbg("port_data_role=%u, port_power_role=%u\n", port_name,
		msg_header.port_data_role,
		msg_header.port_power_role_cable_plug);
	pd_vdbg("msg_type=%d, expected=%d\n", port_name,
		msg_header.msg_type, VDM_DATA_MSG_TYPE_VENDOR_DEFINED);
	/*
	 * The VDM message from Power Adapter must be SOP, Source and
	 * the port partner of the Power Adapter must be DFP.
	 */
	if (sop_type != VDM_SOP_TYPE_SOP || msg_header.data_objs < 1 ||
			!msg_header.port_power_role_cable_plug || size % 4 ||
			msg_header.msg_type !=
				VDM_DATA_MSG_TYPE_VENDOR_DEFINED) {
		pd_vdbg("not Power Adapter sent out VDM data\n", port_name);
		return;
	}

	pa_status = usbc_pdport_get_pa_state(pdport);
	switch (pa_status) {
	case PDPORT_PA_NONE:
		pd_vdbg("pa_status = PDPORT_PA_NONE\n", port_name);
		if (msg_header.data_objs != 1 || !msg_header.port_data_role ||
				pa_vdm_opcode !=
					PA_VDM_OPCODE_REQUEST_RANDOM_DATA) {
			pd_vdbg("vdm data not match:\n", port_name);
			pd_vdbg("  data_objs=%u, expected=1\n", port_name,
				msg_header.data_objs);
			pd_vdbg("  port_data_role=%u, expected=1(DFP)\n",
				port_name, msg_header.port_data_role);
			pd_vdbg("  pa_vdm_opcode=0x%08x, expected=0x%08x\n",
				port_name, pa_vdm_opcode,
				PA_VDM_OPCODE_REQUEST_RANDOM_DATA);
			usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
			break;
		}

		/* Power adapter request random data command received. */
		usbc_pdport_set_pa_state(pdport,
				PDPORT_PA_REQUEST_RANDOM_DATA_RCVD);
		pd_vdbg("request random data command recived\n", port_name);
		cancel_delayed_work(&pdport->delayed_work);

		/* fall-through */
	case PDPORT_PA_REQUEST_RANDOM_DATA_RCVD:
		pd_vdbg("pa_status = PDPORT_PA_REQUEST_RANDOM_DATA_RCVD\n",
			port_name);

		err = usbc_pdport_send_random_data(pdport);
		if (err == -EBUSY) {
			/* Wait for the port status idle. */
			schedule_delayed_work(&pdport->delayed_work,
				msecs_to_jiffies(USBC_PDPORT_BUSY_RETRY_TIME));
			pd_dbg("busying, will be resent later\n", port_name);
		} else {
			pd_dbg("EC sent random data, %d\n", port_name, err);
		}

		break;
	case PDPORT_PA_EC_RANDON_DATA_SENT:
		pd_vdbg("pa_status = PDPORT_PA_EC_RANDON_DATA_SENT\n",
			port_name);
		if (msg_header.data_objs < 2 || !msg_header.port_data_role ||
				pa_vdm_opcode !=
					PA_VDM_OPCODE_ENCRIPTED_RANDOM_DATA) {
			pd_vdbg("vdm data not match:\n", port_name);
			pd_vdbg("  data_objs=%u, expected=2\n", port_name,
				msg_header.data_objs);
			pd_vdbg("  port_data_role=%u, expected=1(DFP)\n",
				port_name, msg_header.port_data_role);
			pd_vdbg("  pa_vdm_opcode=0x%08x, expected=0x%08x\n",
				port_name, pa_vdm_opcode,
				PA_VDM_OPCODE_ENCRIPTED_RANDOM_DATA);
			usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
			break;
		}

		/* Power adapter responds with encripted random data. */
		usbc_pdport_set_pa_state(pdport,
				PDPORT_PA_ENCRIPTED_RANDON_DATA_RCVD);
		pd_vdbg("encripted random data received, PA_opcode=0x%08x\n",
			port_name, pa_vdm_opcode);

		/* fall-through */
	case PDPORT_PA_ENCRIPTED_RANDON_DATA_RCVD:
		pd_vdbg("pa_status = PDPORT_PA_ENCRIPTED_RANDON_DATA_RCVD\n",
			port_name);
		if (!pa_vdm_is_valid_random_data(port_name,
				get_unaligned_le32(&vdm_msg->vdo[1]))) {
			/*
			 * Invalid encripted random data sent by Power Adapter,
			 * so, the Power Adapter cannot be authenticated
			 * successfully, and supported.
			 */
			usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
			pd_vdbg("invalid encripted data, not support\n",
				port_name);
			break;
		}

		/*
		 * The Power Adapter has been authenticated and supported.
		 * So the Power Adapter power tune operation should be
		 * supported.
		 */

		/*
		 * Try to enter DFP mode if not yet running in DPF mode.
		 * The power tune command must be sent as in DFP mode.
		 */
		if (!usbc_pdport_is_ec_dfp_mode(pdport)) {
			/* Send to command to Swap to DFP mode from UFP. */
			pd_vdbg("need to swap to DFP mode\n", port_name);
			err = pdport_ops->data_role_swap(port_name);
			if (err) {
				pd_err("sent data role swap fail, %d\n",
					port_name, err);
				usbc_pdport_set_pa_state(pdport,
							 PDPORT_PA_NONE);
				break;
			}

			pd_vdbg("sent data role swap success\n", port_name);

			usbc_pdport_set_pa_state(pdport,
						 PDPORT_PA_DR_SWAP_SENT);
		} else {
			/* Already in DFP mode, directly goto next stage. */
			pd_vdbg("already in DFP mode\n", port_name);
			usbc_pdport_set_pa_state(pdport, PDPORT_PA_DR_SWAPPED);
		}

		/* fall-through */
	case PDPORT_PA_DR_SWAP_SENT:
		/* fall-through */
	case PDPORT_PA_DR_SWAPPED:
		/* fall-through */
	case PDPORT_PA_POWER_TUNE_READY:
		pa_status = usbc_pdport_get_pa_state(pdport);
		if (pa_status == PDPORT_PA_DR_SWAP_SENT) {
			pd_vdbg("pa_status = PDPORT_PA_DR_SWAP_SENT\n",
				port_name);
			break;	/* Wait for DR_SWAP complete event. */
		}

		usbc_pdport_pa_dr_swap_ready_process(pdport);
		break;
	case PDPORT_PA_POWER_TUNE_CMD_SENT:
		pd_vdbg("pa_status = PDPORT_PA_POWER_TUNE_CMD_SENT\n",
			port_name);
		if (msg_header.data_objs < 2 || msg_header.port_data_role ||
				pa_vdm_opcode !=
					PA_VDM_OPCODE_POWER_TUNE_CMD_RESP) {
			pd_vdbg("vdm data not match, bypass:\n", port_name);
			pd_vdbg("  data_objs=%u, expected=2\n", port_name,
				msg_header.data_objs);
			pd_vdbg("  port_data_role=%u, expected=0(UFP)\n",
				port_name, msg_header.port_data_role);
			pd_vdbg("  pa_vdm_opcode=0x%08x, expected=0x%08x\n",
				port_name, pa_vdm_opcode,
				PA_VDM_OPCODE_POWER_TUNE_CMD_RESP);
			break;
		}

		/* VDM power and current command response received. */
		usbc_pdport_set_pa_state(pdport,
				PDPORT_PA_POWER_TUNE_CMD_RESP_RCVD);

		/* fall-through */
	case PDPORT_PA_POWER_TUNE_CMD_RESP_RCVD:
		pd_vdbg("pa_status = PDPORT_PA_POWER_TUNE_CMD_RESP_RCVD\n",
			port_name);

		power_current.vdo = get_unaligned_le32(&vdm_msg->vdo[1]);
		if (power_current.pa_response.id == PA_VDM_CMD_ID) {
			pd_dbg("success done, correct CMD ID received\n",
				port_name);
		} else {
			pd_dbg("invalid ID received=0x%04x, expected=0x%04x\n",
				port_name, power_current.pa_response.id,
				PA_VDM_CMD_ID);
		}

		if (power_current.pa_response.current_level !=
			pa_vdm_mA_to_current_level(pdport->current_mA) ||
		    power_current.pa_response.voltage !=
			pa_vdm_mV_to_voltage_value(pdport->voltage_mV)) {
			pd_warn("tune cmd resp result not match\n", port_name);
		}
		pd_dbg("latest power voltate=%s, current=%s\n", port_name,
			pa_vdm_voltage_value_to_V_string(
				power_current.pa_response.voltage),
			pa_vdm_current_level_to_A_string(
				power_current.pa_response.current_level));

		/*
		 * A power voltage current tune command process done,
		 * reset the pa_state to ready for next tune command.
		 */
		pdport->voltage_mV = pa_vdm_voltage_value_to_mV(
				power_current.pa_response.voltage);
		pdport->current_mA = pa_vdm_current_level_to_mA(
				power_current.pa_response.current_level);
		if (pdport->send_vdm_sync) {
			pd_dbg("send_vdm_sync, done\n", port_name);
			complete(&pdport->done);
		} else {
			pd_dbg("done, reset to PDPORT_PA_POWER_TUNE_READY\n",
				port_name);
			usbc_pdport_set_pa_state(pdport,
				PDPORT_PA_POWER_TUNE_READY);
			/*
			 * TODO (Customer):
			 * For send tune command in async mode.
			 * New tune command can be sent after here.
			 */
		}
		break;
	default:
		pd_warn("unknown status=%d, reset to PDPORT_PA_NONE\n",
			port_name, (int)usbc_pdport_get_pa_state(pdport));
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
		break;
	}
}

static void usbc_pdport_event_callback(char *port_name,
		u32 event_code, u8 *event_data, size_t size)
{
	struct usbc_pdport *pdport = usbc_pdport_name_to_pdport(port_name);
	struct usbc_pdport_operations *pdport_ops = usbc_pdport_get_ops(pdport);

	if (!pdport || !pdport_ops) {
		pd_warn("invalid port instance, skip\n", port_name);
		return;
	}

	switch (event_code) {
	case PDPORT_TYPE_C_CONNECTED:
		pd_vdbg("PDPORT_TYPE_C_CONNECTED\n", port_name);
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
		break;
	case PDPORT_TYPE_C_DISCONNECTED:
		pd_vdbg("PDPORT_TYPE_C_DISCONNECTED\n", port_name);
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
		break;
	case PDPORT_CONTRACT_ESTABLISHED:
		pd_vdbg("PDPORT_CONTRACT_ESTABLISHED\n", port_name);
		usbc_pdport_set_pa_state(pdport, PDPORT_PA_NONE);
		break;
	case PDPORT_DR_SWAPPED:
		pd_vdbg("PDPORT_DR_SWAPPED\n", port_name);
		if (usbc_pdport_get_pa_state(pdport) ==
				PDPORT_PA_DR_SWAP_SENT) {
			usbc_pdport_set_pa_state(pdport, PDPORT_PA_DR_SWAPPED);
			usbc_pdport_pa_dr_swap_ready_process(pdport);
		}
		break;
	case PDPORT_PR_SWAPPED:
		pd_vdbg("PDPORT_PR_SWAPPED\n", port_name);
		break;
	case PDPORT_VCONN_SWAPPED:
		pd_vdbg("PDPORT_VCONN_SWAPPED\n", port_name);
		break;
	case PDPORT_VCONN_SWITCHED:
		pd_vdbg("PDPORT_VCONN_SWITCHED\n", port_name);
		break;
	case PDPORT_PS_RDY:
		pd_vdbg("PDPORT_PS_RDY\n", port_name);
		break;
	case PDPORT_ACCEPT_RECEIVED:
		pd_vdbg("PDPORT_ACCEPT_RECEIVED\n", port_name);
		break;
	case PDPORT_SOURCE_CAP_RCVD:
		pd_vdbg("PDPORT_SOURCE_CAP_RCVD\n", port_name);
		break;
	case PDPORT_SINK_CAP_RCVD:
		pd_vdbg("PDPORT_SINK_CAP_RCVD\n", port_name);
		break;
	case PDPORT_VDM_RECEIVED:
		pd_vdbg("PDPORT_VDM_RECEIVED\n", port_name);
		usbc_pdport_pa_vdm_event_process(pdport, event_data, size);
		break;
	case PDPORT_CMD_FAIL_ERROR_DETECTED:
		pd_vdbg("PDPORT_CMD_FAIL_ERROR_DETECTED\n", port_name);
		usbc_pdport_pd_vdm_cmd_error_process(pdport);
		break;
	case PDPORT_DRV_REMOVED:
		pd_vdbg("PDPORT_DRV_REMOVED\n", port_name);
		usbc_pdport_deinit(pdport);
		break;
	default:
		pd_vdbg("NOT SUPPORT EVENT CODE=0x%x\n", port_name, event_code);
		break;
	}
}

#if defined(CONFIG_CUSTOMER_PD_TEST) && CONFIG_CUSTOMER_PD_TEST
/*
 * customer_pd_init - A sample pdport instances initialization function.
 * Note, normally the pdport instances initailization function should be applied
 * in the other driver modules, not in this cyccg driver modules.
 * But for the sample demo purpose, this pdport instances initialization
 * function will be called in cyccg driver's probe routine. It's just for
 * demo purpose.
 */
void customer_pd_init(void)
{
	static struct usbc_pdport pdport[USBC_PDPORT_NUM];
	char name[USBC_PDPORT_NAME_SIZE];
	int i;

	pd_vdbg("<<<< enter\n", NULL);

	/* Make sure all the PD ports are initalized only one time. */
	spin_lock(&usbc_pdports_lock);
	if (!list_empty(&usbc_pdports_list)) {
		spin_unlock(&usbc_pdports_lock);
		return;
	}
	spin_unlock(&usbc_pdports_lock);

	for (i = 1; i <= USBC_PDPORT_NUM; i++) {
		scnprintf(name, USBC_PDPORT_NAME_SIZE,
				USBC_PDPORT_NAME_FORMAT, i);
		usbc_pdport_init(&pdport[i], name);
	}

	pd_vdbg(">>>> exit\n", NULL);
}

void customer_pd_uninit(void)
{
	struct usbc_pdport *pdport;

	pd_vdbg("<<<< enter\n", NULL);

	spin_lock(&usbc_pdports_lock);
	list_for_each_entry(pdport, &usbc_pdports_list, node) {
		if (pdport) {
			spin_unlock(&usbc_pdports_lock);
			usbc_pdport_deinit(pdport);
			spin_lock(&usbc_pdports_lock);
		}
	}
	spin_unlock(&usbc_pdports_lock);

	pd_vdbg(">>>> exit\n", NULL);
}
#endif