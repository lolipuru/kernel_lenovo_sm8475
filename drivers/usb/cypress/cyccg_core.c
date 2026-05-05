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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include "cyccg_core.h"
#include "customer_pd.h"


#define cyccg_err(fmt, ...)	ccg_err(fmt, ##__VA_ARGS__)
#define cyccg_warn(fmt, ...)	ccg_warn(fmt, ##__VA_ARGS__)
#define cyccg_info(fmt, ...)	ccg_info(fmt, ##__VA_ARGS__)
#define cyccg_dbg(fmt, ...)	ccg_dbg(fmt, ##__VA_ARGS__)
#define cyccg_vdbg(fmt, ...)	ccg_vdbg(fmt, ##__VA_ARGS__)
#define cyccg_dump(fmt, buf, size, ...)		\
	ccg_dump(fmt, buf, size, ##__VA_ARGS__)

#define cyccg_port_err(fmt, port, ...)	port_err(fmt, port, ##__VA_ARGS__)
#define cyccg_port_warn(fmt, port, ...)	port_warn(fmt, port, ##__VA_ARGS__)
#define cyccgc_port_info(fmt, port, ...)	\
	port_info(fmt, port, ##__VA_ARGS__)
#define cyccg_port_dbg(fmt, port, ...)	port_dbg(fmt, port, ##__VA_ARGS__)
#define cyccg_port_vdbg(fmt, port, ...)	port_vdbg(fmt, port, ##__VA_ARGS__)
#define cyccg_port_dump(fmt, port, buf, size, ...)	\
	port_dump(fmt, port, buf, size, ##__VA_ARGS__)


static int cyccg_device_init(struct cyccg *cyccg);
static irqreturn_t cyccg_device_detect_handler(int irq, void *dev_id);
static irqreturn_t cyccg_irq_thread_handler(int irq, void *dev_id);
static void cyccg_polling_timer_thread_handler(struct work_struct *work);

/**
 *****************************************************************************
 * Following functions are the key points that will be various on different
 * platforms and different board devices, os need to be configured or modified
 * based on the real device configurations.
 * Please pay attention to the comments under all "TODO(Customer):".
 *****************************************************************************
 */
static int cyccg_dt_parse(struct device *dev, struct cyccg_platform_data *pdata)
{
	struct device_node *dev_node = dev->of_node;
	u32 value;
	int err;

	err = of_property_read_u32(dev_node, "dp_mode", &value);
	if (err || (value != (u32)DP_SOURCE && value != (u32)DP_SINK))
		value = (u32)DP_UNKNOWN;
	pdata->dp_mode = (enum hpi_dp_mode)value;

	pdata->reset_gpio =
		of_get_named_gpio_flags(dev_node, "reset_gpio", 0, NULL);
	pdata->intr_gpio =
		of_get_named_gpio_flags(dev_node, "intr_gpio", 0, NULL);

	/*
	 * TODO(Customer):
	 * Add more DT node parse here if defined and used on the platform.
	 */

	cyccg_dbg("dp_mode = %s\n", (pdata->dp_mode == DP_SOURCE) ?
		"DP_SOURCE" : (pdata->dp_mode == DP_SINK ?
					"DP_SINK" : "DP_UNKNOWN"));
	cyccg_dbg("reset_gpio = %d\n", pdata->reset_gpio);
	cyccg_dbg("intr_gpio = %d\n", pdata->intr_gpio);

	return 0;
}

static void cyccg_predefined_dp_mode_configure(struct cyccg *cyccg)
{
	/*
	 * TODO(Customer):
	 * Set the current pre-defined DP_MODE if not defined in DT.
	 */
	cyccg->ccg_info.dp_mode = cyccg->platform_data ?
			cyccg->platform_data->dp_mode : DP_UNKNOWN;
	cyccg_dbg("predefined dp_mode = %s\n",
		cyccg->ccg_info.dp_mode == DP_SOURCE ? "DP_SOURCE" :
			(cyccg->ccg_info.dp_mode == DP_SINK) ?
				"DP_SINK" : "DP_UNKNOWN");
}

static int cyccg_gpio_init(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct cyccg_platform_data *pdata = cyccg->platform_data;
	int err;

	cyccg_vdbg("init gpio\n");

	/* NO requirement to initialize the GPIO pins. */
	if (!pdata) {
		cyccg_dbg("no platform data set, skip\n");
		return 0;
	}


	/* gpio for CCGx chip reset. */
	if (pdata->reset_gpio > 0) {
		err = devm_gpio_request(dev,
				pdata->reset_gpio, "cyccg_reset_gpio");
		if (err) {
			cyccg_err("failed to request gpio=%d, %d\n",
				pdata->reset_gpio, err);
			return err;
		}

		/*
		 * CCGx chip reset must firstly pull the gpio low at least 10ms
		 * then pull up.
		 * Default is High to enable the CCGx device.
		 */
		gpio_direction_output(pdata->reset_gpio, 1);
		msleep(30);
	}

	if (pdata->intr_gpio > 0) {
		err = devm_gpio_request(dev,
				pdata->intr_gpio, "cyccg_intr_gpio");
		if (err) {
			cyccg_err("failed to request gpio=%d, %d\n",
				pdata->intr_gpio, err);
			return err;
		}

		/*
		 * Interrupt pin default is High.
		 * Low indicates interrupt from CCGx.
		 */
		gpio_direction_input(pdata->intr_gpio);
		cyccg->irq = gpio_to_irq(pdata->intr_gpio);
		if (cyccg->irq < 0) {
			cyccg_err("failed to map intr_gpio to irq, %d\n",
				err);
			return -EINVAL;
		}
	}

	/*
	 * TODO(Customer):
	 * Add more gpio pin initialize and release code based on board design.
	 */

	return 0;
}

int cyccg_port_init(struct hpi_device *port)
{
	struct hpi_pd_src_sink_pdo_list pdo_list;
	u32 event_mask;
	u8 pdo_mask;
	int err;

	cyccg_port_vdbg(">>>> enter\n", port);

	/*
	 * TODO (Customer):
	 * Change the event_mask values if some events were not required.
	 * Default enable all events to be reported to host.
	 */
	event_mask = 0xffffffff;
	err = hpi_port_write_event_mask_sync(port, event_mask);
	if (err) {
		cyccg_port_err("failed to set event_mask, %d\n",
			port, err);
		return err;
	}

	/*
	 * TODO (Customer):
	 * Change the pdo_mask value if other Source PDOs were support.
	 * or for CCG3/CCG4, a dynamic PDO list can be supplied to
	 * work together with the pdo_mask.
	 */
	pdo_mask = 0x01;
	memset(&pdo_list, 0, sizeof(struct hpi_pd_src_sink_pdo_list));
#if 1	/* Sample code to set Source PDO mask. */
	err = hpi_port_select_source_pdo_sync(port, pdo_mask, NULL, 0);
#else
	pdo_list.signature_pdo_data_type = HPI_SIGNATURE_SELECT_SOURCE_PDO;
	/* Initialize Source pdo_list.pdos here. */
	err = hpi_port_select_source_pdo_sync(port, pdo_mask,
		&pdo_list, sizeof(struct hpi_pd_src_sink_pdo_list));
#endif
	if (err) {
		cyccg_port_err("failed to select Source PDO mask, %d\n",
			port, err);
		return err;
	}

	/*
	 * TODO (Customer):
	 * Change the pdo_mask value if other Sink PDOs were support.
	 * or for CCG3/CCG4, a dynamic PDO list can be supplied to
	 * work together with the pdo_mask.
	 */
	pdo_mask = 0x01;
#if 1	/* Sample code to set Sink PDO mask. */
	err = hpi_port_select_sink_pdo_sync(port, pdo_mask, NULL, 0);
#else
	memset(&pdo_list, 0, sizeof(struct hpi_pd_src_sink_pdo_list));
	pdo_list.signature_pdo_data_type = HPI_SIGNATURE_SELECT_SOURCE_PDO;
	/* Initialize Sink pdo_list.pdos here. */
	err = hpi_port_select_sink_pdo_sync(port, pdo_mask,
		&pdo_list, sizeof(struct hpi_pd_src_sink_pdo_list));
#endif
	if (err) {
		cyccg_port_err("failed to select Sink PDO mask, %d\n",
			port, err);
		return err;
	}

	err = hpi_port_ec_initialization_complete_sync(port);
	if (err) {
		cyccg_port_err("failed to complete EC init, %d\n",
			port, err);
		return err;
	}

	spin_lock(&port->slock);
	port->enabled = true;
	spin_unlock(&port->slock);
	cyccg_port_vdbg(">>>> exit, port init success\n", port);
	return 0;
}

/*
 * cyccg_pmic_control - notify external PMIC to update the power
 *	input/output and voltage and current power status.
 * @port: the handle of the PD port.
 * @source: when true, indicates host is source, the VBUS should be turn on;
 *	when false, the host is sink, the VBUS should turn off.
 * @voltage_mV: indicates the PMIC to output or accept the power voltage in mV.
 *	When the value of voltage_mV < 0, it should be ignored.
 * @current_mA: indicates the PMIC to output or accept the power current in mA.
 *	When the value of current_mA < 0, it should be ignored.
 */
static int cyccg_pmic_control(struct hpi_device *port,
		bool is_source, int voltage_mV, int current_mA)
{
	char *port_name = port->pdport_name_addr->name;
	int err = 0;

	cyccg_port_vdbg(">>>> enter\n", port);

	port_name = port_name;	/* Suppress compile warning. */
	if (is_source) {
		cyccg_port_dbg("set CCG as power Source device, <%dmV, %dmA>\n",
			port, voltage_mV, current_mA);
		/*
		 * TODO (Customer):
		 * Notify PMIC to turn VBUS on, set the correct voltage and
		 * current value based on the input values of voltage_mV and
		 * current_mA, and other required status.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
	} else {
		cyccg_port_dbg("set CCG as power Sink device, <%dmV, %dmA>\n",
			port, voltage_mV, current_mA);
		/*
		 * TODO (Customer):
		 * Notify PMIC to turn VBUS off, set the correct voltage and
		 * current value based on the input values of voltage_mV and
		 * current_mA, and other required status.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
	}

	return err;
}

/*
 * cyccg_usb_mode_control - notify USB module to update the USB port as host
 *	or device mode.
 * @port: the handle of the PD port.
 * @usb_mode: indicates the USB device mode should be set to.
 *	USB_DRP = USB host and device mode;
 *	USB_DFP = USB host mode;
 *	USB_UFP = USB device mode.
 */
static int cyccg_usb_mode_control(struct hpi_device *port,
		enum usbc_pdport_mode usb_mode)
{
	char *port_name = port->pdport_name_addr->name;
	int err = 0;

	cyccg_port_vdbg(">>>> enter\n", port);

	port_name = port_name;	/* Suppress compile warning. */
	switch (usb_mode) {
	case USB_DRP:
		cyccg_port_dbg("set usb_mode to USB_DRP\n", port);
		/*
		 * TODO (Customer):
		 * Notify the USB module to set the USB port to OTG host and
		 * device mode, and other required status.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
		break;
	case USB_DFP:
		cyccg_port_dbg("set usb_mode to USB_DFP\n", port);
		/*
		 * TODO (Customer):
		 * Notify the USB module to set the USB port to OTG host mode
		 * and other required status.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
		break;
	case USB_UFP:
		cyccg_port_dbg("set usb_mode to USB_UFP\n", port);
		/*
		 * TODO (Customer):
		 * Notify the USB module to set the USB port to OTG device mode
		 * and other required status.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
		break;
	case USB_DEBUG:
		cyccg_port_dbg("set usb_mode to USB_DEBUG\n", port);
		/*
		 * TODO (Customer):
		 * Notify the USB module to set to debug accessory mode.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
		break;
	case USB_AUDIO:
		cyccg_port_dbg("set usb_mode to USB_AUDIO\n", port);
		/*
		 * TODO (Customer):
		 * Notify the USB module to set to audio accessory mode.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
		break;
	case USB_POWERED:
		cyccg_port_dbg("set usb_mode to USB_POWERED\n", port);
		/*
		 * TODO (Customer):
		 * Notify the USB module to set to power accessory mode.
		 * When mulit-ports exists, the port_name can be sent and used
		 * to identify which USB Type-C port it command should be sent
		 * to.
		 */
		break;
	default:
		cyccg_port_dbg("error: unknown usb_mode = %d\n",
			port, (int)usb_mode);
		return -EINVAL;
	}

	return err;
}

/*
 * cyccg_pmic_usb_mode_sync - used to notify external PMIC and USB
 *	modules to update the Power role and Data role status based on latest
 *	port Type-C and PD status.
 * @port - instance handle of the PD port.
 * @port_status - pointer to the latest port Type-C and PD status.
 */
static int cyccg_pmic_usb_mode_sync(struct hpi_device *port,
		struct usbc_pdport_status *port_status)
{
	int voltage_mV = -1;
	int current_mA = -1;
	int err = 0;

	cyccg_port_vdbg(">>>> enter\n", port);

	/*
	 * No Type-C connected or PD contract established and
	 * Attached device removed, so turn VBUS off, set to DRP and other
	 * status.
	 */
	if (!port_status->type_c_connected &&
			!port_status->contract_established) {
		cyccg_port_dbg("cable unplugged\n", port);

		err = cyccg_pmic_control(port,
				false, voltage_mV, current_mA);
		if (!err) {
			err = cyccg_usb_mode_control(port, USB_DRP);
			if (err)
				cyccg_port_err("set USB mode failed, %d\n",
					port, err);
		} else {
			cyccg_port_err("set PMIC failed, %d\n", port, err);
		}

		return err;
	}

	switch (port_status->attached_device_type) {
	case ATTACHED_DEV_TYPE_SINK:
		cyccg_port_dbg("attached device type is Sink\n", port);

		/* Attached is Sink, so set host as Source and DFP. */
		err = cyccg_pmic_control(port,
				true, voltage_mV, current_mA);
		if (!err) {
			err = cyccg_usb_mode_control(port, USB_DFP);
			if (err)
				cyccg_port_err("set USB mode failed, %d\n",
					port, err);
		} else {
			cyccg_port_err("set PMIC failed, %d\n", port, err);
		}

		break;
	case ATTACHED_DEV_TYPE_SOURCE:
		cyccg_port_dbg("attached device type is Source\n", port);

		/* Attached is Source, so set host as Sink and UFP. */
		err = cyccg_pmic_control(port,
				false, voltage_mV, current_mA);
		if (!err) {
			err = cyccg_usb_mode_control(port, USB_UFP);
			if (err)
				cyccg_port_err("set USB mode failed, %d\n",
					port, err);
		} else {
			cyccg_port_err("set PMIC failed, %d\n", port, err);
		}

		break;
	case ATTACHED_DEV_TYPE_DEBUG_ACCESSORY:
		cyccg_port_dbg("attached device type is Debug Accessory\n",
			port);

		err = cyccg_usb_mode_control(port, USB_DEBUG);
		if (err)
			cyccg_port_err("set USB mode failed, %d\n", port, err);
		break;
	case ATTACHED_DEV_TYPE_AUDIO_ACCESSORY:
		cyccg_port_dbg("attached device type is Audio Accessory\n",
			port);

		err = cyccg_usb_mode_control(port, USB_AUDIO);
		if (err)
			cyccg_port_err("set USB mode failed, %d\n", port, err);

		break;
	case ATTACHED_DEV_TYPE_POWERED_ACCESSORY:
		cyccg_port_dbg("attached device type is Powered Accessory\n",
			port);

		err = cyccg_usb_mode_control(port, USB_POWERED);
		if (err)
			cyccg_port_err("set USB mode failed, %d\n", port, err);

		break;
	case ATTACHED_DEV_TYPE_UNSUPPORTED_ACCESSORY:
		cyccg_port_dbg("attached dev type is Unsupported Accessory\n",
			port);
		break;
	default:
		cyccg_port_dbg("nothing or unknown attached_dev_type=%d\n",
			port, (int)port_status->attached_device_type);
		break;
	}

	return err;
}

static inline int cyccg_pdport_hard_reset_event_process(struct hpi_device *port)
{
	struct cyccg *cyccg = port->cyccg;
	unsigned long flags;
	unsigned long irq_passed_time;
	int wait_time;
	int voltage_mV = -1;
	int current_mA = -1;
	int err;

	cyccg_port_vdbg(">>>> enter\n", port);
	if (!port->status.current_power_role) {
		/* Set to USB OTG device (UFP) mode. */
		err = cyccg_usb_mode_control(port, USB_UFP);
		if (err)
			cyccg_port_warn("set USB mode to UFP error, %d\n",
				port, err);
		return err;
	}

	/*
	 * 1. Wait 25-35ms for the HARD_RESET command prepare ready.
	 *    When the wait time has passed based on slow irq thread scheudle,
	 *    then bypass the prepare ready wait.
	 */
	spin_lock_irqsave(&cyccg->polling_timer_slock, flags);
	irq_passed_time = jiffies - cyccg->last_irq_time;
	spin_unlock_irqrestore(&cyccg->polling_timer_slock, flags);
	wait_time = 25000 - jiffies_to_usecs(irq_passed_time);
	if (wait_time > 0)
		usleep_range(wait_time, wait_time + 10000);

	/* 2. Turn VBus off */
	err = cyccg_pmic_control(port, false, voltage_mV, current_mA);
	if (err) {
		cyccg_port_warn("turn VBUS off error, %d\n", port, err);
		return err;
	}

	/* 3. Wait 700-740ms for the HARD_RESET completed. */
	usleep_range(700000, 740000);

	/* 4. Set to USB OTG host (DFP) mode. */
	err = cyccg_usb_mode_control(port, USB_DFP);
	if (err) {
		cyccg_port_warn("set USB mode to DFP error, %d\n", port, err);
		return err;
	}

	/* 5. Turn VBus on */
	err = cyccg_pmic_control(port, true, voltage_mV, current_mA);
	if (err) {
		cyccg_port_warn("turn VBUS on error, %d\n", port, err);
		return err;
	}

	return 0;
}

/*
 * cyccg_update_pmic_usb_mode_status - used to notify external PMIC and USB
 *	module to update the Power role and Data role status.
 * @port - instance handle of the PD port.
 * @msg - the hpi message data read from CCG device.
 *	  When @msg is NULL, then force update the USB mode with port status.
 */
static int cyccg_update_pmic_usb_mode_status(struct hpi_device *port,
					     struct hpi_msg *msg)
{
	struct hpi_swap_status *swap_status;
	struct usbc_pdport_status port_status;
	enum hpi_swap_type swap_type;
	u32 event_code = msg ? msg->code : HPI_PD_RESP_PD_CONTRACT_ESTABLISHED;
	int voltage_mV = -1;
	int current_mA = -1;
	bool retried = false;
	bool host_do_swap;
	int err = 0;

	/*
	 * CCG FW will sync Power and data role status to PMIC and USB mode,
	 * so thid driver do noting in this situation.
	 */
	if (!IS_ENABLED(CONFIG_CONTROL_PMIC_USB_MODE))
		return 0;

	cyccg_port_vdbg("<<<< enter\n", port);
	memset(&port_status, 0, sizeof(port_status));
	switch (event_code) {
	case HPI_PD_RESP_PD_CONTRACT_ESTABLISHED:
		cyccg_port_dbg("event_code = %s\n",
			port, msg ? "HPI_PD_RESP_PD_CONTRACT_ESTABLISHED" :
					"NULL, force update");

		/* Force sync with PMIC and USB mode. */
		err = hpi_port_get_port_status(port, &port_status, false);
		if (err) {
			cyccg_port_err("failed to read port status, %d\n",
				port, err);
			break;
		}

		err = cyccg_pmic_usb_mode_sync(port, &port_status);
		if (err) {
			cyccg_port_err("failed to sync PMIC and USB mode, %d\n",
				port, err);
			break;
		}

		memcpy(&port->status, &port_status,
					sizeof(struct usbc_pdport_status));
		break;
	case HPI_PD_RESP_TYPE_C_CONNECTED:
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
		cyccg_port_dbg("event_code = %s\n",
			port, event_code == HPI_PD_RESP_TYPE_C_CONNECTED ?
				"HPI_PD_RESP_TYPE_C_CONNECTED" :
				"HPI_PD_RESP_TYPE_C_DISCONNECTED");
		do {
			err = hpi_port_get_port_status(port,
						&port_status, false);
			if ((msg->code == HPI_PD_RESP_TYPE_C_CONNECTED &&
					!port_status.type_c_connected) ||
			    (msg->code == HPI_PD_RESP_TYPE_C_DISCONNECTED &&
					(port_status.type_c_connected ||
					 port_status.contract_established)))
				err = err ?: -EINVAL;

			if (!err || retried)
				break;

			/* Wait the registers to be updated. */
			usleep_range(1000, 2000);
			retried = true;
		} while (err);
		if (err) {
			/*
			 * This situation will be happpen when doing
			 * quickly unplug and replug test.
			 * The cable has been re-pluged or re-unplugged
			 * again befer this status read opeartion. And
			 * the CONNECTED or DISCONECTED event has been
			 * queued in the CCG message queue, so the
			 * value of these CCG registers have been
			 * updated. So stop retrying and try to do
			 * next events processing instead.
			 */
			break;
		}

		memcpy(&port->status, &port_status,
					sizeof(struct usbc_pdport_status));
		err = cyccg_pmic_usb_mode_sync(port, &port->status);
		if (err) {
			cyccg_err("failed to sync PMIC and USB mode, %d\n",
				err);
			break;
		}

		break;
	case HPI_PD_RESP_SWAP_COMPLETE:
		cyccg_port_dbg("event_code = HPI_PD_RESP_SWAP_COMPLETE\n",
			port);
		spin_lock(&port->slock);
		host_do_swap = port->is_swap_triggered_by_host;
		spin_unlock(&port->slock);
		swap_status = (struct hpi_swap_status *)msg->data;
		swap_type = (enum hpi_swap_type)swap_status->swap_type;

		switch (swap_type) {
		case HPI_DR_SWAP:
			cyccg_port_dbg("%s triggered DR_SWAP completed\n", port,
				host_do_swap ? "EC Host" : "Port Partner");
			if (port->status.current_data_role ==
					PD_PORT_DATA_ROLE_DFP) {
				/*
				 * CCG is DFP, so Swap the CCG power
				 * role to UFP.
				 */
				err = cyccg_usb_mode_control(port, USB_UFP);
				if (err)
					cyccg_port_err("set to USB_UFP, %d\n",
						port, err);
			} else {
				/*
				 * CCG is UFP, so Swap the CCG power
				 * role to DFP.
				 */
				err = cyccg_usb_mode_control(port, USB_DFP);
				if (err)
					cyccg_port_err("set to USB_DFP, %d\n",
						port, err);
			}
			break;
		case HPI_PR_SWAP:
			cyccg_port_dbg("%s triggered PR_SWAP completed\n", port,
				host_do_swap ? "EC Host" : "Port Partner");
			if (!host_do_swap && port->status.current_power_role ==
					PD_PORT_POWER_ROLE_SOURCE) {
				/*
				 * PR_SWAP initiated by Port Partner,
				 * and CCG is Source, so Swap the CCG power
				 * role to Sink.
				 *
				 * Note, when PR_SWAP event is received
				 * by the driver, the latest register value of
				 * the PD_STATUS has already become invaild,
				 * cannot be used, so use last record PD status.
				 */
				err = cyccg_pmic_control(port,
					false, voltage_mV, current_mA);
				if (err)
					cyccg_port_err("set PMIC fail, %d\n",
						port, err);
			} else {
				/*
				 * CCG is Sink, so do nothing,
				 * waiting for Port Partner's PS_RDY event.
				 */
			}
			break;
		case HPI_VCONN_SWAP:
			cyccg_port_dbg("HPI_VCONN_SWAP\n", port);
			break;
		default:
			cyccg_port_dbg("swap_type = %d\n", port, swap_type);
			break;
		}

		break;
	case HPI_PD_RESP_PS_RDY:
		cyccg_port_dbg("event_code = HPI_PD_RESP_PS_RDY\n", port);
		spin_lock(&port->slock);
		host_do_swap = port->is_swap_triggered_by_host;
		spin_unlock(&port->slock);
		cyccg_port_dbg("PS_RDY received, SWAP cmd triggered by %s\n",
			port, host_do_swap ? "EC Host" : "Port Partner");

		if (port->status.current_power_role ==
					PD_PORT_POWER_ROLE_SOURCE) {
			cyccg_port_dbg("current_power_role = SOURCE\n", port);
			/*
			 * Host CCG is source.
			 * When the Port Partner tirggered PR_SWAP, do nothing,
			 * the power role must been set to sink after the
			 * PR_SWAP complete event received.
			 * When the host CCG triggered PR_SWAP, do nothing,
			 * the power role must be set to sink after the ACCEPT
			 * message was received.
			 */
		} else {
			cyccg_port_dbg("current_power_role = SINK\n", port);
			/*
			 * CCG is Sink, after the PS_RDY received from Port
			 * Partner, the CCG then can swap power role to Source.
			 */
			err = cyccg_pmic_control(port,
				true, voltage_mV, current_mA);
			if (err)
				cyccg_port_err("set PMIC fail, %d\n",
					port, err);
		}

		break;
	case HPI_PD_RESP_ACCEPT_MESSAGE:
		cyccg_port_dbg("event_code = HPI_PD_RESP_ACCEPT_MESSAGE\n",
			port);
		spin_lock(&port->slock);
		host_do_swap = port->is_swap_triggered_by_host;
		spin_unlock(&port->slock);

		cyccg_port_dbg("host_do_swap = %d\n", port, host_do_swap);
		if (host_do_swap && port->status.current_power_role ==
					PD_PORT_POWER_ROLE_SOURCE) {
			cyccg_port_dbg("current_power_role = SOURCE\n", port);
			/*
			 * The PR_SWAP initiated by CCG host, and CCG is source.
			 * accepted by Port Partner, Swap to Sink.
			 */
			err = cyccg_pmic_control(port,
				false, voltage_mV, current_mA);
			if (err)
				cyccg_port_err("set PMIC fail, %d\n",
					port, err);
		}

		break;
	case HPI_PD_RESP_HARD_RESET:
	case HPI_PD_HARD_RESET_SENT:
		cyccg_port_dbg("event_code = %s\n",
			port, event_code == HPI_PD_RESP_HARD_RESET ?
				"HPI_PD_RESP_HARD_RESET" :
				"HPI_PD_HARD_RESET_SENT");
		err = cyccg_pdport_hard_reset_event_process(port);
		break;
	case HPI_PD_UNEXPECTED_VOLTAGE_VBUS:
		cyccg_port_dbg("event_code = HPI_PD_UNEXPECTED_VOLTAGE_VBUS\n",
			port);
		err = cyccg_pmic_control(port, false, voltage_mV, current_mA);
		if (err)
			cyccg_port_err("turn VBUS off error, %d\n", port, err);
	}

	cyccg_port_vdbg(">>>> exit\n", port);
	return err;
}

/*
 * TODO (Customer):
 * Pre-initiallize the PD Port names with the CCG Bus, address and port index
 * map based on the physical board/system desgion.
 *
 * All the USB-C PD Port name should be pre-initialized in the data
 * @usbc_pdport_name_addr_map. It's aimed to help get the same PD Port name for
 * the same USB-C PD Port on same CCG device each time.
 */
static struct usbc_pdport_name_addr
usbc_pdport_name_addr_map[USBC_PDPORT_NUM + 1] = {
	{ USBC_PDPORT_NAME(1), BUS_I2C, 8, 0x08, 0, NULL },
	{ USBC_PDPORT_NAME(2), 0, 0, 0, 0, NULL  },
	{ USBC_PDPORT_NAME(3), 0, 0, 0, 0, NULL  },
	{ USBC_PDPORT_NAME(4), 0, 0, 0, 0, NULL  },
	{ USBC_PDPORT_NAME(5), 0, 0, 0, 0, NULL  },
	{ USBC_PDPORT_NAME(6), 0, 0, 0, 0, NULL  },
	{ USBC_PDPORT_NAME(7), 0, 0, 0, 0, NULL  },
	{ USBC_PDPORT_NAME(8), 0, 0, 0, 0, NULL  },
/*
 ****************************************************************************
 * Folowing are the sample code to set the port name and addr map.
 * Please the real USB-C PD Port name and address map in above this comments.
 ****************************************************************************
 *	{ USBC_PDPORT_NAME(1), BUS_I2C, 6, 0x08, 0, NULL  },
 *	{ USBC_PDPORT_NAME(2), BUS_I2C, 6, 0x08, 1, NULL  },
 *
 *	{ USBC_PDPORT_NAME(3), BUS_I2C, 7, 0x08, 0, NULL  },
 *
 *	{ USBC_PDPORT_NAME(4), BUS_I2C, 8, 0x08, 0, NULL  },
 *
 *	{ USBC_PDPORT_NAME(5), BUS_SPI, 4, 0x08, 0, NULL  },
 *
 *	{ USBC_PDPORT_NAME(6), BUS_SPI, 4, 0x40, 0, NULL  },
 ****************************************************************************
 */
};

/*///////////////////////////////////////////////////////////////////////////*/

static DEFINE_SPINLOCK(usbc_pdport_list_lock);
static LIST_HEAD(usbc_pdport_list);
static int usbc_pdport_count;

static void cyccg_usbc_pdport_register(struct hpi_device *port)
{
	struct ccg_bus_operations *bus_ops = port->cyccg->bus_ops;
	struct usbc_pdport_name_addr *pdport_name_addr;
	u16 bustype = bus_ops->bustype;
	u16 bus = bus_ops->bus;
	u32 addr = bus_ops->addr;
	int port_index = port->idx;
	int empty_slot = -1;
	int i;

	cyccg_vdbg("<<<< enter, port->idx=%d\n", port->idx);
	if (port->dev_type <= HPI_DEV_TYPE_DEVICE) {
		cyccg_port_warn("not USB-C PD Port device, skip. dev_type=%d\n",
			port, (int)port->dev_type);
		return;
	}

	spin_lock(&usbc_pdport_list_lock);

	for (i = 0; i < USBC_PDPORT_NUM; i++) {
		pdport_name_addr = &usbc_pdport_name_addr_map[i];

		/* Encountered not initialized name, exit. */
		if (!pdport_name_addr->name ||
				strlen(pdport_name_addr->name) == 0) {
			cyccg_warn("name_addr_map uninit port name, i=%d\n", i);
			break;
		}

		/*
		 * Get the PD Port name set but address not initialized slot.
		 * This slot can be used to register the address unmapped
		 * CCG device and port.
		 */
		if (!pdport_name_addr->dev) {
			if (empty_slot < 0)
				empty_slot = i;
			continue;
		}

		if (pdport_name_addr->bustype == bustype &&
				pdport_name_addr->bus == bus &&
				pdport_name_addr->addr == addr &&
				pdport_name_addr->port_index == port_index) {
			port->pdport_name_addr = pdport_name_addr;
			pdport_name_addr->dev = port->cyccg->dev;
			goto success;
		}
	}

	if (unlikely(empty_slot < 0)) {
		cyccg_warn("USB-C PD Port num exceeds max USBC_PDPORT_NUM=%d\n",
			USBC_PDPORT_NUM);
		goto out;
	}

	/* Register the address unmapped CCG device and port. */
	pdport_name_addr = &usbc_pdport_name_addr_map[empty_slot];
	pdport_name_addr->bustype = bustype;
	pdport_name_addr->bus = bus;
	pdport_name_addr->addr = addr;
	pdport_name_addr->port_index = port_index;
	pdport_name_addr->dev = port->cyccg->dev;
	port->pdport_name_addr = pdport_name_addr;
success:
	list_add(&port->node, &usbc_pdport_list);
	usbc_pdport_count++;
	cyccg_port_dbg(">>>> register port_name=%s, %s, %u, 0x%x, %d\n",
		port, pdport_name_addr->name,
		pdport_name_addr->bustype == BUS_I2C ? "BUS_I2C" :
			pdport_name_addr->bustype == BUS_SPI ? "BUS_SPI" :
				"BUS_UNKNOWN",
		pdport_name_addr->bus, pdport_name_addr->addr, port_index);
out:
	spin_unlock(&usbc_pdport_list_lock);
}

static void cyccg_usbc_pdport_unregister(struct hpi_device *port)
{
	cyccg_port_vdbg("<<<< enter\n", port);
	if (port->dev_type <= HPI_DEV_TYPE_DEVICE) {
		cyccg_warn("not USB-C PD Port device, skip. dev_type=%d\n",
			(int)port->dev_type);
		return;
	}

	spin_lock(&usbc_pdport_list_lock);

	list_del_init(&port->node);
	usbc_pdport_count--;
	port->pdport_name_addr->dev = NULL;
	port->pdport_name_addr = NULL;

	spin_unlock(&usbc_pdport_list_lock);
}

static struct hpi_device *cyccg_get_port_by_name(const char *port_name)
{
	struct hpi_device *port;

	if (!port_name)
		return NULL;

	spin_lock(&usbc_pdport_list_lock);
	list_for_each_entry(port, &usbc_pdport_list, node) {
		if (port && !strncasecmp(port_name,
					port->pdport_name_addr->name,
					USBC_PDPORT_NAME_SIZE))
			goto out;
	}

	port = NULL;	/* Not found */
out:
	spin_unlock(&usbc_pdport_list_lock);
	return port;
}

/*
 * cyccg_get_usbc_pdport_name_addr_by_name - Retrieve a valid pre-defined USB-C
 *	PD Port name and address data structure based on the port name.
 * @port_name: the pre-defined name of the Type-C port index.
 *
 * Ruturn the pointer to the valid struct usbc_pdport_name_addr data;
 * otherwise, NULL will be returned.
 */
struct usbc_pdport_name_addr *cyccg_get_usbc_pdport_name_addr_by_name(
		char *port_name)
{
	struct usbc_pdport_name_addr *pdport_name_addr;
	int i;

	if (!port_name || !strlen(port_name)) {
		cyccg_warn("invalid port_name=%s\n", port_name ?: "NULL");
		return NULL;
	}

	spin_lock(&usbc_pdport_list_lock);

	for (i = 0;  i < USBC_PDPORT_NUM; i++) {
		pdport_name_addr = &usbc_pdport_name_addr_map[i];
		if (pdport_name_addr->dev &&
				!strncasecmp(port_name, pdport_name_addr->name,
					USBC_PDPORT_NAME_SIZE)) {
			goto out;
		}
	}

	pdport_name_addr = NULL;
out:
	spin_unlock(&usbc_pdport_list_lock);
	return pdport_name_addr;
}
EXPORT_SYMBOL(cyccg_get_usbc_pdport_name_addr_by_name);


/*
 * cyccg_get_usbc_pdport_name_by_addr - Retrieve a valid pre-defined USB-C
 *	PD Port name and address data structure based on the port bus type,
 *	bus number, bus address and the Type-C port index.
 * @bustype: the value of bus type the CCG device attached, such as BUS_I2C or
 *	BUS_SPI.
 * @bus: the bus number of the bus @bustype in the board/system desgin.
 * @addr: the address of the CCG device appeared on the @bus.
 * @port_index: the USB-C port index in same CCG device. Started from 0.
 *
 * Ruturn the pointer to the valid struct usbc_pdport_name_addr data;
 * otherwise, NULL will be returned.
 */
struct usbc_pdport_name_addr *cyccg_get_usbc_pdport_name_addr_by_addr(
		u16 bustype, u16 bus, u32 addr, int port_index)
{
	struct usbc_pdport_name_addr *pdport_name_addr;
	int i;

	spin_lock(&usbc_pdport_list_lock);

	for (i = 0;  i < USBC_PDPORT_NUM; i++) {
		pdport_name_addr = &usbc_pdport_name_addr_map[i];
		if (pdport_name_addr->dev &&
				pdport_name_addr->bustype == bustype &&
				pdport_name_addr->bus == bus &&
				pdport_name_addr->addr == addr &&
				pdport_name_addr->port_index == port_index) {
			goto out;
		}
	}

	pdport_name_addr = NULL;
out:
	spin_unlock(&usbc_pdport_list_lock);
	return pdport_name_addr;
}
EXPORT_SYMBOL(cyccg_get_usbc_pdport_name_addr_by_addr);

int cyccg_get_port_list_string(struct cyccg *cyccg, char *buf, size_t size)
{
	struct usbc_pdport_name_addr *name_addr;
	struct hpi_device *port;
	int i;

	spin_lock(&usbc_pdport_list_lock);

	size = scnprintf(buf, size,
		"Port_Name	Bus_Type	Bus	Addr	Port_Id\n");

	for (i = 0; i < USBC_PDPORT_NUM; i++) {
		name_addr = &usbc_pdport_name_addr_map[i];

		if (!name_addr->name || strlen(name_addr->name) == 0 ||
				!name_addr->dev || !name_addr->bustype)
			continue;

		list_for_each_entry(port, &usbc_pdport_list, node) {
			if (port && !strncasecmp(name_addr->name,
					port->pdport_name_addr->name,
					USBC_PDPORT_NAME_SIZE))
				break;
		}

		size += scnprintf(buf + size, PAGE_SIZE - size,
			"%s	%s		%u	0x%04x	%d\n",
			name_addr->name,
			name_addr->bustype == BUS_I2C ? "I2C" :
				name_addr->bustype == BUS_SPI ? "SPI" :
					"UNKNOWN",
			name_addr->bus, name_addr->addr,
			name_addr->port_index);
	}

	spin_unlock(&usbc_pdport_list_lock);
	return size;
}

/*//////////////////////////////////////////////////////////////////////////*/

/*
 * cyccg_get_port_status - Retrieve the USB-C PD Port's Type-C and PD status
 *	specified by the port name.
 * @port_name: indicates the name of the USB-C PD port which to retrieve status.
 * @port_status: points to the data structure of the memory to return the
 *	retrieve PD Port status.
 */
static int cyccg_get_port_status(char *port_name,
		struct usbc_pdport_status *port_status)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);
	int err;

	if (!port || !port_status) {
		cyccg_err("unknown USB-C PD Port name=%s or invalid params\n",
			port_name ?: "NULL");
		return -EINVAL;
	}

	err = hpi_port_get_port_status(port, port_status, true);
	if (err) {
		cyccg_port_err("failed to read port status, %d\n", port, err);
		return err;
	}

	return 0;
}

static void usbc_pdport_send_vdm_cmd_callback(struct hpi_device *port,
		void *param, int errcode, void *data, size_t size)
{
	char *port_name = port->pdport_name_addr->name;

	if (errcode) {
		cyccg_port_dbg("send vdm command failed, %d\n", port, errcode);
		mutex_lock(&port->event_cb_mlock);
		if (port->event_callback)
			port->event_callback(port_name,
				(u32)PDPORT_CMD_FAIL_ERROR_DETECTED,
				(u8 *)&errcode, sizeof(errcode));
		mutex_unlock(&port->event_cb_mlock);
		return;
	}

	mutex_lock(&port->event_cb_mlock);
	if (port->event_callback) {
		port->event_callback(port_name,
			PDPORT_VDM_RECEIVED, data, size);
	}
	mutex_unlock(&port->event_cb_mlock);
}

static int cyccg_usbc_pdport_send_vdm(char *port_name,
		u8 *vdm_cmd_data, size_t size)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);
	int err;

	cyccg_vdbg("<<<< enter, port_name=%s\n", port_name ?: "NULL");
	if (!port || !vdm_cmd_data || !size || size % VDM_VDO_OBJ_SIZE) {
		cyccg_err("unknown USB-C PD Port name=%s or invalid params\n",
			port_name ?: "NULL");
		return -EINVAL;
	}

	if (cyccg_busying_check_and_set(port)) {
		cyccg_port_warn("CC port is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	cyccg_port_dump("send vdm data(%zu):\n", port, vdm_cmd_data, size,
		size);

	err = hpi_port_send_vdm_data_async(port, VDM_SOP_TYPE_SOP,
			vdm_cmd_data, size,
			usbc_pdport_send_vdm_cmd_callback, NULL);
	if (err) {
		cyccg_port_err("send_vdm command failed, %d\n", port, err);
		usbc_pdport_send_vdm_cmd_callback(port, NULL, err, NULL, 0);
		goto out;
	}

out:
	cyccg_set_to_busying_state(port, false);
	cyccg_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

static void usbc_pdport_role_swap_callback(struct hpi_device *port,
		void *param, int errcode, void *data, size_t size)
{
	char *port_name = port->pdport_name_addr->name;
	enum usbc_pdport_event_code event_code;
	enum hpi_swap_type swap_type;

	hpi_cmd_get_async_callback_param(port, NULL, param);
	swap_type = (enum hpi_swap_type)(unsigned long)param;
	switch (swap_type) {
	case HPI_DR_SWAP:
		event_code = PDPORT_DR_SWAPPED;
		break;
	case HPI_PR_SWAP:
		event_code = PDPORT_PR_SWAPPED;
		break;
	case HPI_VCONN_SWAP:
		event_code = PDPORT_VCONN_SWAPPED;
		break;
	default:
		cyccg_port_warn("port swap_type=%d data not match\n", port,
			(int)swap_type);
		return;
	}

	if (errcode) {
		cyccg_port_err("vdm command failed, %d\n", port, errcode);
		mutex_lock(&port->event_cb_mlock);
		if (port->event_callback)
			port->event_callback(port_name,
				(u32)PDPORT_CMD_FAIL_ERROR_DETECTED,
				(u8 *)&errcode, sizeof(errcode));
		mutex_unlock(&port->event_cb_mlock);
		return;
	}

	mutex_lock(&port->event_cb_mlock);
	if (port->event_callback)
		port->event_callback(port_name, event_code, data, size);
	mutex_unlock(&port->event_cb_mlock);
}

static int cyccg_usbc_pdport_data_role_swap(char *port_name)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);
	int err;

	cyccg_port_vdbg("<<<< enter\n", port);
	if (!port) {
		cyccg_err("unknown USB-C PD Port name=%s or invalid params\n",
			port_name ?: "NULL");
		return -EINVAL;
	}

	if (cyccg_busying_check_and_set(port)) {
		cyccg_port_warn("CC port is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_data_role_swap_async(port,
			usbc_pdport_role_swap_callback, (void *)HPI_DR_SWAP);
	if (err) {
		cyccg_port_err("send_vdm command failed, %d\n", port, err);
		usbc_pdport_role_swap_callback(port, (void *)HPI_DR_SWAP,
			err, NULL, 0);
		goto out;
	}

out:
	cyccg_set_to_busying_state(port, false);
	cyccg_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

static int cyccg_usbc_pdport_power_role_swap(char *port_name)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);
	int err;

	cyccg_port_vdbg("<<<< enter\n", port);
	if (!port) {
		cyccg_err("unknown USB-C PD Port name=%s or invalid params\n",
			port_name ?: "NULL");
		return -EINVAL;
	}

	if (cyccg_busying_check_and_set(port)) {
		cyccg_port_warn("CC port is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_power_role_swap_async(port,
			usbc_pdport_role_swap_callback, (void *)HPI_PR_SWAP);
	if (err) {
		cyccg_port_err("send_vdm command failed, %d\n", port, err);
		usbc_pdport_role_swap_callback(port, (void *)HPI_PR_SWAP,
			err, NULL, 0);
		goto out;
	}

out:
	cyccg_set_to_busying_state(port, false);
	cyccg_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

static int cyccg_usbc_pdport_vconn_swap(char *port_name)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);
	int err;

	cyccg_port_vdbg("<<<< enter\n", port);
	if (!port) {
		cyccg_err("unknown USB-C PD Port name=%s or invalid params\n",
			port_name ?: "NULL");
		return -EINVAL;
	}

	if (cyccg_busying_check_and_set(port)) {
		cyccg_port_warn("CC port is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_vconn_role_swap_async(port,
			usbc_pdport_role_swap_callback, (void *)HPI_VCONN_SWAP);
	if (err) {
		cyccg_port_err("send_vdm command failed, %d\n", port, err);
		usbc_pdport_role_swap_callback(port, (void *)HPI_VCONN_SWAP,
			err, NULL, 0);
		goto out;
	}

out:
	cyccg_set_to_busying_state(port, false);
	cyccg_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

static void usbc_pdport_vconn_switch_callback(struct hpi_device *port,
		void *param, int errcode, void *data, size_t size)
{
	char *port_name = port->pdport_name_addr->name;

	hpi_cmd_get_async_callback_param(port, NULL, param);

	if (errcode) {
		cyccg_port_warn("vdm command failed, %d\n", port, errcode);
		mutex_lock(&port->event_cb_mlock);
		if (port->event_callback)
			port->event_callback(port_name,
				(u32)PDPORT_CMD_FAIL_ERROR_DETECTED,
				(u8 *)&errcode, sizeof(errcode));
		mutex_unlock(&port->event_cb_mlock);
		return;
	}

	mutex_lock(&port->event_cb_mlock);
	if (port->event_callback)
		port->event_callback(port_name, (u32)PDPORT_VCONN_SWITCHED,
				(u8 *)param, sizeof(void *));
	mutex_unlock(&port->event_cb_mlock);
}

static int cyccg_usbc_pdport_switch_vconn(char *port_name, bool on)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);
	unsigned long param = (unsigned long)!!on;
	int err;

	cyccg_port_vdbg("<<<< enter\n", port);
	if (!port) {
		cyccg_err("unknown USB-C PD Port name=%s or invalid params\n",
			port_name ?: "NULL");
		return -EINVAL;
	}

	if (cyccg_busying_check_and_set(port)) {
		cyccg_port_warn("CC port is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_switch_vconn_async(port, on,
		usbc_pdport_vconn_switch_callback, (void *)param);
	if (err) {
		cyccg_port_err("send_vdm command failed, %d\n", port, err);
		usbc_pdport_vconn_switch_callback(port, (void *)param,
			err, NULL, 0);
		goto out;
	}

out:
	cyccg_set_to_busying_state(port, false);
	cyccg_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

static struct usbc_pdport_operations usbc_pdport_ops = {
	.get_port_status = cyccg_get_port_status,
	.send_vdm = cyccg_usbc_pdport_send_vdm,
	.data_role_swap = cyccg_usbc_pdport_data_role_swap,
	.power_role_swap = cyccg_usbc_pdport_power_role_swap,
	.vconn_swap = cyccg_usbc_pdport_vconn_swap,
	.vconn_switch = cyccg_usbc_pdport_switch_vconn,
};

int cyccg_usbc_pdport_event_register(char *port_name,
		struct usbc_pdport_operations **pdport_ops,
		usbc_pdport_event_cb_t usbc_pdport_event_callback)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);

	if (!port || !pdport_ops || !usbc_pdport_event_callback) {
		cyccg_port_err("invalid parameteres, port_name=%s\n",
			port, port_name);
		return -EINVAL;
	}

	*pdport_ops = &usbc_pdport_ops;
	mutex_lock(&port->event_cb_mlock);
	port->event_callback = usbc_pdport_event_callback;
	mutex_unlock(&port->event_cb_mlock);

	cyccg_port_vdbg("registered success, event_callback=0x%p\n",
		port, port->event_callback);
	return 0;
}
EXPORT_SYMBOL(cyccg_usbc_pdport_event_register);

void cyccg_usbc_pdport_event_unregister(char *port_name)
{
	struct hpi_device *port = cyccg_get_port_by_name(port_name);

	cyccg_vdbg("<<<< enter, unregister port_name=%s\n", port_name);
	if (port) {
		mutex_lock(&port->event_cb_mlock);
		port->event_callback = NULL;
		mutex_unlock(&port->event_cb_mlock);
		cyccg_port_vdbg("unregistered success\n", port);
	}
}
EXPORT_SYMBOL(cyccg_usbc_pdport_event_unregister);

/*
 * cyccg_hpi_msg_code_to_customer_pd_code - remap the HPI message code to
 *	customer internal defined PD code.
 * @hpi_msg_code: CCGx reported HPI message code
 * @hpi_msg_data: CCGx reported HPI message data. It can be NULL if not requied.
 *
 * Return customer internal defined PD code if the HPI message code was
 * successfully mapped. If no map relation existing, then invalid value U32_MAX
 * value will be returned. The caller must check the return value.
 * TODO (Customer):
 *	Change the map relation-ship if the PD code is changed.
 */
static inline u32 cyccg_hpi_msg_code_to_customer_pd_code(
		u32 hpi_msg_code, void *hpi_msg_data)
{
	struct hpi_swap_status *swap_status;
	enum hpi_swap_type swap_type;
	u32 customer_pd_code = U32_MAX;

	switch (hpi_msg_code) {
	case HPI_PD_RESP_TYPE_C_CONNECTED:
		customer_pd_code = PDPORT_TYPE_C_CONNECTED;
		break;
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
		customer_pd_code = PDPORT_TYPE_C_DISCONNECTED;
		break;
	case HPI_PD_RESP_PD_CONTRACT_ESTABLISHED:
		customer_pd_code = PDPORT_CONTRACT_ESTABLISHED;
		break;
	case HPI_PD_RESP_PS_RDY:
		customer_pd_code = PDPORT_PS_RDY;
		break;
	case HPI_PD_RESP_ACCEPT_MESSAGE:
		customer_pd_code = PDPORT_ACCEPT_RECEIVED;
		break;
	case HPI_PD_RESP_SRC_CAP_RCVD:
		customer_pd_code = PDPORT_SOURCE_CAP_RCVD;
		break;
	case HPI_PD_RESP_SINK_CAP_RCVD:
		customer_pd_code = PDPORT_SINK_CAP_RCVD;
		break;
	case HPI_PD_RESP_VDM_RECEIVED:
		customer_pd_code = PDPORT_VDM_RECEIVED;
		break;
	case HPI_PD_RESP_SWAP_COMPLETE:
		if (!hpi_msg_data) {
			cyccg_warn("set SWAP_COMPLETE to default DR_SWAPPED\n");
			customer_pd_code = PDPORT_DR_SWAPPED;
			break;
		}

		swap_status = (struct hpi_swap_status *)hpi_msg_data;
		swap_type = (enum hpi_swap_type)swap_status->swap_type;
		switch (swap_type) {
		case HPI_DR_SWAP:
			customer_pd_code = PDPORT_DR_SWAPPED;
			break;
		case HPI_PR_SWAP:
			customer_pd_code = PDPORT_PR_SWAPPED;
			break;
		case HPI_VCONN_SWAP:
			customer_pd_code = PDPORT_VCONN_SWAPPED;
			break;
		default:
			break;
		}

		break;
	default:
		break;
	}

	return customer_pd_code;
}

static void cyccg_run_usbc_pdport_callback(struct hpi_device *port,
				    struct hpi_msg *msg)
{
	u32 event_code;
	char *port_name;

	cyccg_port_vdbg("<<<< enter\n", port);
	if (!port || port->dev_type <= HPI_DEV_TYPE_DEVICE)
		return;

	event_code =
		cyccg_hpi_msg_code_to_customer_pd_code(msg->code, msg->data);
	if (event_code == U32_MAX)	/* Invalid mapped customer PD code. */
		return;

	cyccg_port_dbg("HPI msg->code=0x%x\n", port, msg->code);
	cyccg_port_dbg("callback event code=0x%x\n", port, event_code);
	mutex_lock(&port->event_cb_mlock);

	port_name = port->pdport_name_addr->name;
	if (port->event_callback)
		port->event_callback(port_name,
				event_code, msg->data, msg->len);

	mutex_unlock(&port->event_cb_mlock);
	cyccg_port_vdbg(">>>> exit\n", port);
}

/*//////////////////////////////////////////////////////////////////////////*/

void cyccg_set_to_busying_state(struct hpi_device *hpidev, bool busying)
{
	spin_lock(&hpidev->slock);
	hpidev->busying = busying;
	spin_unlock(&hpidev->slock);
}

static bool _cyccg_is_busying(struct hpi_device *hpidev)
{
	struct cyccg *cyccg = hpidev->cyccg;
	enum hpi_device_type dev_type = hpidev->dev_type;
	struct hpi_device *port;
	bool busying;
	int i;

	if (dev_type == HPI_DEV_TYPE_DEVICE) {
		spin_lock(&hpidev->slock);
		busying = hpidev->busying;
		spin_unlock(&hpidev->slock);
		if (busying)
			return busying;	/* Host CCG FW update running. */

		for (i = 0; i < cyccg->ccg_info.num_port; i++) {
			port = cyccg->ports[i];
			if (!port || port->dev_type != HPI_DEV_TYPE_PORT)
				continue;

			spin_lock(&port->slock);
			busying = port->busying;
			spin_unlock(&port->slock);
			if (busying)
				break;	/* Port Partner FW update running. */
		}
	} else {
		spin_lock(&hpidev->slock);
		busying = hpidev->busying;
		spin_unlock(&hpidev->slock);
		if (!busying) {
			if (dev_type != HPI_DEV_TYPE_MIXED) {
				spin_lock(&cyccg->hpi_dev.slock);
				busying = cyccg->hpi_dev.busying;
				spin_unlock(&cyccg->hpi_dev.slock);
			}
		}
	}

	return busying;
}

bool cyccg_busying_check_and_set(struct hpi_device *hpidev)
{
	struct cyccg *cyccg = hpidev->cyccg;
	bool is_busying;

	mutex_lock(&cyccg->mlock);

	is_busying = _cyccg_is_busying(hpidev);
	if (!is_busying)
		cyccg_set_to_busying_state(hpidev, true);

	mutex_unlock(&cyccg->mlock);

	return is_busying;
}

void cyccg_wait_for_idle(struct hpi_device *hpidev)
{
	struct cyccg *cyccg = hpidev->cyccg;

	mutex_lock(&cyccg->mlock);

	while (_cyccg_is_busying(hpidev))
		msleep(CYCCG_FW_UPDATE_WAIT_CHECK_INTERVAL);

	cyccg_set_to_busying_state(hpidev, true);

	mutex_unlock(&cyccg->mlock);
}

enum ccg_version ccg_silicon_id_to_ccg_version(u16 silicon_id)
{
	switch (silicon_id & CCGX_SILICON_ID_MASK) {
	case CCG1_SILICON_ID_MASK:
		return CCG1;
	case CCG2_SILICON_ID_MASK:
		return CCG2;
	case CCG3_SILICON_ID_MASK:
		return CCG3;
	case CCG4_SILICON_ID_MASK:
		return CCG4;
	default:
		break;
	}

	cyccg_err("unknown silicon id: 0x%04x\n", silicon_id);
	return CCG_UNKNOWN;
}

static int cyccg_dt_parse_and_get_pdata(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct cyccg_platform_data *pdata;
	int err;

	if (!dev->of_node) {
		cyccg_warn("no of_node data existing\n");
		return 0;
	}

	pdata = devm_kzalloc(dev,
			     sizeof(struct cyccg_platform_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	err = cyccg_dt_parse(dev, pdata);
	if (err) {
		devm_kfree(dev, pdata);
		cyccg_err("failed to parse device of_node data, %d\n", err);
		return err;
	}

	cyccg_predefined_dp_mode_configure(cyccg);

	cyccg->platform_data = pdata;
	dev->platform_data = pdata;
	return 0;
}

static irqreturn_t cyccg_device_detect_handler(int irq, void *dev_id)
{
	struct cyccg *cyccg = dev_id;
	struct device *dev = cyccg->dev;
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	u8 cmd[] = { HPI_SIGNATURE_ENTER_FLASHING_MODE, 0 };
	size_t size = HPI_REG_SIZE_OF(ENTER_FLASHING_MODE, HPI_VERSION_1);
	u32 addr = HPI_REG_OFFSET_OF(ENTER_FLASHING_MODE, HPI_VERSION_1);
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	unsigned long end_time;
	struct hpi_v1_msg msg;
	int err;

	/*
	 * When the first IRQ event received, directly force the possible
	 * CCG device enter flashing mode to avoid the Boot Wait Windows (32ms)
	 * is missed. It's aimed to ensure the CCG device stay in bootloader
	 * mode and the I2C communication is active for later process.
	 *
	 * Note, this is only for on CCG1 and CCG2 devices which support
	 * HPIv1.
	 */
	cyccg_dbg("<<<< enter, force jump to bootlaoder mode\n");
	err = bus_ops->write(dev, HPI_REG_ADDR_SIZE(HPI_VERSION_1),
				cmd, size, addr);
	if (err) {
		cyccg_err("failed to write enter flashing mode cmd, %d\n", err);
		goto out;
	}

	/* Wait for the command response, until the Boot Wait Window timeout. */
	end_time = jiffies + msecs_to_jiffies(32);
	size = HPI_REG_SIZE_OF(DEVICE_RESPONSE, HPI_VERSION_1);
	addr = HPI_REG_OFFSET_OF(DEVICE_RESPONSE, HPI_VERSION_1);
	while (time_before(jiffies, end_time)) {
		memset(&msg, 0, sizeof(struct hpi_v1_msg));
		err = bus_ops->read(dev, HPI_REG_ADDR_SIZE(HPI_VERSION_1),
					&msg, size, addr);
		if (err) {
			cyccg_err("failed to read DEVICE_RESPONSE, %d\n", err);
			goto out;
		}

		if (msg.code == HPI_PD_RESP_NO_RESPONSE) {
			usleep_range(1000, 2000);
			continue;
		}

		err = hpi_clear_intr(hpidev);
		if (err) {
			cyccg_err("failed to clear interrupt bit, %d\n", err);
			goto out;
		}

		if (msg.code == HPI_PD_RESP_SUCCESS) {
			cyccg_dbg("enter flashing mode success\n");
			break;
		}

		cyccg_dump("message data (code=0x%02x, len=%u):\n",
			msg.data, msg.len, msg.code, msg.len);
	}

out:
	complete(&cyccg->done);
	cyccg_dbg(">>>> exit\n");
	return IRQ_HANDLED;
}

static int cyccg_device_detect(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	struct cyccg_platform_data *pdata = cyccg->platform_data;
	unsigned long timeout;
	int retries = 5;
	int err;

	cyccg_vdbg("<<<< enter\n");
	/*
	 * In following situations, the I2C communication would be not
	 * accessible for a little time, so a retry is requied here to double
	 * check the device can be detected correctly.
	 *   1) CCG device is booting, and before the bootloader I2C module
	 *      be actived.
	 *   2) CCG device is changing from bootloader to applicaton image.
	 *      The I2C communication cannot be accessed after the bootloader
	 *      I2C module deactived, and before the application i2C module
	 *      is actived.
	 */
	while (retries--) {
		err = bus_ops->detect(dev);
		if (!err)
			return 0;

		msleep(100);
	}

	/*
	 * After the retry, still failed to detect the CCG device.
	 * Try to reset the CCG device to force detect it if it support.
	 * If still no CCG device detected, then give up.
	 */
	cyccg_dbg("no CCG device detected, try further detect\n");
	if (!pdata || !pdata->reset_gpio) {
		cyccg_dbg("platform data or CCGx reset GPIO not found, %d\n",
			  err);
		return err;
	}

	/* Release XRES pin to disable CCG device for preparing HARD_RESET. */
	gpio_direction_output(pdata->reset_gpio, 0);
	usleep_range(500, 1000);

	/* Register the specific irq thread handler for CCG device detect. */
	err = request_threaded_irq(cyccg->irq,
				   NULL, cyccg_device_detect_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "cyccg_device_detect", cyccg);
	if (err) {
		cyccg_err("no CCG, failed to register irq thread, %d\n", err);
		return err;
	}

	/*
	 * Re-assert the XRES pin to HARD_RESET the CCG device, then wait for
	 * the RESET_COMPLETE event. It should be always the first irq event.
	 */
	init_completion(&cyccg->done);
	gpio_direction_output(pdata->reset_gpio, 1);

	timeout = HPI_TIME_OF(JUMP_TO_BOOT, HPI_VERSION_2);
	timeout = wait_for_completion_timeout(&cyccg->done,
					      msecs_to_jiffies(timeout));
	if (timeout == 0)
		cyccg_warn("wait irq from CCG device timeout\n");

	/* Double check the CCG device for the last time. */
	err = bus_ops->detect(dev);
	err = err ? -ENODEV : 0;

	free_irq(cyccg->irq, cyccg);
	cyccg_dbg("CCG device detected result: %d\n", err);
	return err;
}

static inline void cyccg_hpidev_init(struct cyccg *cyccg,
				     struct hpi_device *hpidev)
{
	struct hpi_command *hpicmd = &hpidev->cmd;

	hpidev->cyccg = cyccg;

	mutex_init(&hpidev->mlock);
	spin_lock_init(&hpidev->slock);
	INIT_WORK(&hpidev->cc_fw_work, cyccg_port_cc_fw_update_work);

	INIT_LIST_HEAD(&hpidev->node);
	mutex_init(&hpidev->event_cb_mlock);

	mutex_init(&hpicmd->mlock);
	spin_lock_init(&hpicmd->slock);
	INIT_DELAYED_WORK(&hpicmd->timer, hpi_cmd_timeout_function);
}

static int cyccg_get_basic_ccg_info(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	struct ccg_info *ccg_info = &cyccg->ccg_info;
	struct hpi_device_mode device_mode;
	int err;

	cyccg_vdbg("<<<< enter\n");
	if (unlikely(!bus_ops)) {
		cyccg_err("invalid NULL bus_ops pointer\n");
		return -EINVAL;
	}

	err = bus_ops->read(dev, HPI_REG_ADDR_SIZE(HPI_VERSION_1),
			    &ccg_info->silicon_id, sizeof(u16),
			    HPI_V1_REG_OFFSET_OF_READ_SILICON_ID);
	if (err) {
		cyccg_err("failed to read silicon ID, %d\n", err);
		return err;
	}

	ccg_info->ccg_ver = ccg_silicon_id_to_ccg_version(ccg_info->silicon_id);
	if (ccg_info->ccg_ver == CCG_UNKNOWN) {
		cyccg_err("Silicon_Id=0x%04x CCGx dev not supported, exit\n",
			ccg_info->silicon_id);
		return -ENOTSUPP;
	}

	err = bus_ops->read(dev, HPI_REG_ADDR_SIZE(HPI_VERSION_1),
			    &device_mode, sizeof(struct hpi_device_mode),
			    HPI_V1_REG_OFFSET_OF_DEVICE_MODE);
	if (err) {
		cyccg_err("failed to read device_mode, %d\n", err);
		return err;
	}

	ccg_info->hpi_ver = (device_mode.hpi_ver == 0) ?
				HPI_VERSION_1 : HPI_VERSION_2;
	ccg_info->num_port = device_mode.num_port + 1;
	ccg_info->num_port = ccg_info->num_port > HPI_MAX_PD_PORTS ?
		HPI_MAX_PD_PORTS : ccg_info->num_port;
	ccg_info->flash_row_size = (device_mode.flash_row_size == 0) ?
		HPI_V1_REG_SIZE_OF_FLASH_RW : HPI_V2_REG_SIZE_OF_FLASH_RW;
	ccg_info->flash_mode = (ccg_info->hpi_ver == HPI_VERSION_1) ?
		CCG_LEGACY_BOOT_MODE : CCG_DUAL_FW_MODE;

	cyccg_info("CCG silicon id = 0x%04x\n", ccg_info->silicon_id);
	cyccg_info("CCG ccg_ver = CCG%d\n", (int)ccg_info->ccg_ver);
	cyccg_info("CCG hpi_ver = %s\n",
		ccg_info->hpi_ver == HPI_VERSION_1 ?
			"HPI_VERSION_1" : "HPI_VERSION_2");
	cyccg_info("CCG num_port = %u\n", ccg_info->num_port);
	cyccg_info("CCG flash_row_size = %u\n", ccg_info->flash_row_size);
	cyccg_info("CCG flash_mode = %s\n",
		ccg_info->flash_mode == CCG_LEGACY_BOOT_MODE ?
			"CCG_LEGACY_BOOT_MODE" : "CCG_DUAL_FW_MODE");
	return  0;
}

static int cyccg_hpidev_allocate_init(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct ccg_info *ccg_info = &cyccg->ccg_info;
	struct hpi_device *hpidev;
	size_t size;
	int i;
	int err;

	cyccg_vdbg("<<<< enter\n");
	err = cyccg_get_basic_ccg_info(cyccg);
	if (err) {
		cyccg_err("failed to read basic ccg_info data, %d\n", err);
		return err;
	}

	/* cyccg data structure init. */
	mutex_init(&cyccg->mlock);
	spin_lock_init(&cyccg->slock);
	INIT_WORK(&cyccg->device_init_work, cyccg_device_init_work);
	INIT_WORK(&cyccg->fw_work, cyccg_auto_fw_update_work);
	INIT_DELAYED_WORK(&cyccg->polling_timer,
			  cyccg_polling_timer_thread_handler);
	spin_lock_init(&cyccg->polling_timer_slock);
	mutex_init(&cyccg->sysfs_mlock);

	/* Initialize HPI device instance. */
	hpidev = &cyccg->hpi_dev;
	hpidev->dev_type = HPI_DEV_TYPE_DEVICE;
	hpidev->reg_base_addr =
		HPI_REG_OFFSET_OF(DEVICE_BASE, ccg_info->hpi_ver);
	hpidev->reg_map_size =
		HPI_REG_SIZE_OF(DEVICE_BASE, ccg_info->hpi_ver);

	size = 2 * MEM_ALLIGNED_SIZE(HPI_REG_RW_SIZE(ccg_info->hpi_ver));
	hpidev->cmd.msg_buf.head = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!hpidev->cmd.msg_buf.head)
		return -ENOMEM;
	hpidev->cmd.msg_buf.size = HPI_REG_RW_SIZE(ccg_info->hpi_ver);

	hpidev->cmd.copy_buf.head = hpidev->cmd.msg_buf.head +
		MEM_ALLIGNED_SIZE(hpidev->cmd.msg_buf.size);
	hpidev->cmd.copy_buf.size = HPI_REG_RW_SIZE(ccg_info->hpi_ver);

	cyccg_hpidev_init(cyccg, hpidev);

	/* Initialize HPI port instances. */
	if (ccg_info->hpi_ver == HPI_VERSION_1) {
		/* The device and the only port share same register map. */
		hpidev->dev_type = HPI_DEV_TYPE_MIXED;
		hpidev->idx = 0;

		cyccg_usbc_pdport_register(hpidev);
		cyccg->ports[0] = hpidev;
		return 0;
	}

	/* HPIv2 based CCGx has separated port register map. */
	for (i = 0; i < ccg_info->num_port; i++) {
		/*
		 * Allocated memory for hpi_device port instance, and
		 * hpi_buffer for msg_buf, and copy_buf.
		 */
		size = MEM_ALLIGNED_SIZE(sizeof(struct hpi_device)) +
			2 * MEM_ALLIGNED_SIZE(
				HPI_REG_RW_SIZE(ccg_info->hpi_ver));
		hpidev = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!hpidev)
			return -ENOMEM;

		hpidev->dev_type = HPI_DEV_TYPE_PORT;
		hpidev->idx = i;
		hpidev->reg_base_addr = (i + 1) *
			HPI_REG_OFFSET_OF(PORT_BASE, ccg_info->hpi_ver);
		hpidev->reg_map_size =
			HPI_REG_SIZE_OF(PORT_BASE, ccg_info->hpi_ver);

		hpidev->cmd.msg_buf.head = (u8 *)hpidev +
			MEM_ALLIGNED_SIZE(sizeof(struct hpi_device));
		hpidev->cmd.msg_buf.size = HPI_REG_RW_SIZE(ccg_info->hpi_ver);

		hpidev->cmd.copy_buf.head = hpidev->cmd.msg_buf.head +
			MEM_ALLIGNED_SIZE(hpidev->cmd.msg_buf.size);
		hpidev->cmd.copy_buf.size = HPI_REG_RW_SIZE(ccg_info->hpi_ver);

		cyccg_hpidev_init(cyccg, hpidev);
		cyccg_usbc_pdport_register(hpidev);
		cyccg->ports[i] = hpidev;
	}

	return 0;
}

static int cyccg_dump_all_pending_events(struct cyccg *cyccg)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	union hpi_intr_reg intr_reg;
	u8 intr_mask = ~(0xff << (cyccg->ccg_info.num_port + 1));
	int count = 0;
	int err;

	if (hpi_ver == HPI_VERSION_1)
		intr_mask = 0x01;

	do {
		err = hpi_read_intr_reg(cyccg, &intr_reg);
		if (err) {
			cyccg_err("failed to read intr reg, %d\n", err);
			return err;
		}

		if (!(intr_reg.val & intr_mask)) {
			if (count)
				break;
			count++;
			continue;
		}

		intr_reg.val = intr_mask;
		err = hpi_write_intr_reg(cyccg, &intr_reg);
		if (err) {
			cyccg_err("failed to clear intr reg, %d\n", err);
			return err;
		}
	} while (true);

	return 0;
}

int cyccg_device_reset_and_init(struct cyccg *cyccg, bool force_reset)
{
	struct ccg_info *ccg_info = &cyccg->ccg_info;
	struct hpi_boot_mode_reason boot_mode_reason;
	struct hpi_device_mode device_mode;
	int i;
	int err;

	cyccg_vdbg("<<<< enter\n");
	if (force_reset) {
		cyccg_dbg("force_reset enabled\n");
		err = hpi_device_reset_sync(cyccg);
		if (err) {
			cyccg_err("failed to force reset device, %d\n", err);

			/* Try to update the device running_mode state. */
			if (!hpi_read_device_mode(cyccg, &device_mode))
				cyccg->ccg_state.running_mode =
						device_mode.running_mode;
			cyccg_dbg("running_mode = %u\n",
				device_mode.running_mode);
			goto out;
		}
	}

	/*
	 * Make sure the CCGx device has boot into FW. Avoid any possible
	 * issue that, when CCGx was just power on or not ready or in mode
	 * switching stage, which will cause the device regiter map unaccessable
	 * issue.
	 */
	msleep(HPI_TIME_OF(BOOT_INTO_FW, ccg_info->hpi_ver));

	/*
	 * Ensure the CCGx initialization to be done when it's running in
	 * application mode.
	 * If it running in BL mode, dump its reason.
	 */
	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		cyccg_err("failed to read device_mode, %d\n", err);
		return err;
	}

	cyccg_dbg("running_mode = %u\n", device_mode.running_mode);
	if (device_mode.running_mode == CCG_FW_MODE_TYPE_BOOTLAODER) {
		cyccg_warn("CCGx running in boot loader mode\n");
		/* The value of bl_last_row data only can be read in BL mode. */
		if (!ccg_info->bl_last_row_num)
			hpi_read_boot_loader_last_row(cyccg,
					&ccg_info->bl_last_row_num);

		err = hpi_device_reset_sync(cyccg);
		if (err)
			cyccg_err("failed to do device reset, %d\n", err);

		err = hpi_read_device_mode(cyccg, &device_mode);
		if (err) {
			cyccg_err("failed to read device_mode, %d\n", err);
			goto out;
		}

		cyccg_dbg("running_mode = %u\n", device_mode.running_mode);
		if (device_mode.running_mode == CCG_FW_MODE_TYPE_BOOTLAODER) {

			err = hpi_read_boot_mode_reason(cyccg,
					&boot_mode_reason);
			if (err) {
				cyccg_err("read boot_mode_reason err, %d\n",
					err);
				goto out;
			}

			if (boot_mode_reason.boot_mode_request)
				cyccg_warn("in BL mode, JUMP_TO_BOOT\n");
			if (boot_mode_reason.config_table_status)
				cyccg_warn("in BL mode, config tbl invalid\n");
			if (boot_mode_reason.fw_app_1_status)
				cyccg_warn("in BL mode, App image 1 invalid\n");
			if (ccg_info->hpi_ver != HPI_VERSION_1 &&
					boot_mode_reason.fw_app_2_status)
				cyccg_warn("in BL mode, App image 2 invalid\n");
			goto out;
		}
	}

	/*
	 * Initialize echo PD port configuration, set event mask, select default
	 * Source PDOs and Sink PDOs to be used when PD contract was
	 * established.
	 */
	for (i = 0; i < ccg_info->num_port; i++) {
		cyccg_dbg("init port_%d\n", i);
		err = cyccg_port_init(cyccg->ports[i]);
		if (err) {
			cyccg_port_err("Port_%d, failed to do port_init, %d\n",
				cyccg->ports[i], i, err);
			goto out;
		} else {
			cyccg_port_dbg("Port_%d, port_init success, %d\n",
				cyccg->ports[i], i, err);
		}
	}

out:
	/*
	 * Try to update CCGx FW versions and relative info after initialized.
	 * Ignore errors, because these values has little affect on later
	 * functions.
	 */
	hpi_read_all_version(cyccg, &cyccg->ccg_state.ccg_fw_vers);
	if (ccg_info->hpi_ver != HPI_VERSION_1) {
		hpi_read_fw2_version(cyccg, &cyccg->ccg_state.ccg_fw_vers);
		hpi_read_fw_binary_location(cyccg,
			&ccg_info->fw1_start_row_num,
			&ccg_info->fw2_start_row_num);
	}

	cyccg_vdbg(">>>> exit, %d\n", err);
	return err;
}

static int cyccg_device_init(struct cyccg *cyccg)
{
	int err;

	cyccg_vdbg("<<<< enter\n");
	err = cyccg_device_reset_and_init(cyccg, false);
	if (err)
		cyccg_err("failed to do device init, %d\n", err);
	cyccg_vdbg(">>>> exit, %d\n", err);
	return err;
}

void cyccg_device_init_work(struct work_struct *work)
{
	struct cyccg *cyccg =
			container_of(work, struct cyccg, device_init_work);
	int retries = CYCCG_DEVICE_INIT_RETRIES;
	int err;

	cyccg_vdbg("<<<< enter\n");
	spin_lock(&cyccg->slock);
	cyccg->device_init_work_state = CYCCG_WORK_STATE_RUNNING;
	spin_unlock(&cyccg->slock);

	cyccg_wait_for_idle(&cyccg->hpi_dev);
	cyccg_dbg("cyccg_wait_for_idle success\n");

	do {
		err = cyccg_device_init(cyccg);
		if (!err) {
			cyccg_dbg("device init work done success\n");
			break;
		}

		msleep(CYCCG_DEVICE_INIT_RETRIES_INTERVAL);
		cyccg_dbg("device init failed, %d. Retries=%d\n",
			err, CYCCG_DEVICE_INIT_RETRIES - retries + 1);
	} while (err && --retries);

	cyccg_set_to_busying_state(&cyccg->hpi_dev, false);

	spin_lock(&cyccg->slock);
	cyccg->device_init_work_state = CYCCG_WORK_STATE_NONE;
	spin_unlock(&cyccg->slock);
	cyccg_vdbg(">>>> exit, %d\n", err);
}

void cyccg_queue_device_init_work(struct cyccg *cyccg)
{
	/*
	 * The device init work only actived after probe done, avoid to
	 * confused with normal startup device init porcess, so the power on
	 * boot up RESET_COMPLETED event won't miss-trigger the device re-init.
	 */
	spin_lock(&cyccg->slock);
	if (cyccg->device_init_work_state == CYCCG_WORK_STATE_NONE &&
			cyccg->probe_done) {
		cyccg->device_init_work_state = CYCCG_WORK_STATE_QUEUED;
		schedule_work(&cyccg->device_init_work);
	}
	spin_unlock(&cyccg->slock);

	if (cyccg->probe_done)
		cyccg_dbg("CCGx device_init work has been %s\n",
			cyccg->device_init_work_state ? "queued" : "running");
}

static void cyccg_polling_timer_thread_handler(struct work_struct *work)
{
	struct delayed_work *dwork =
			container_of(work, struct delayed_work, work);
	struct cyccg *cyccg = container_of(dwork, struct cyccg, polling_timer);
	unsigned long last_irq_time;
	unsigned long interval;
	unsigned long flags;

	cyccg_vdbg("==== polling timer thread\n");

	cyccg_irq_thread_handler(cyccg->irq, cyccg);

	spin_lock(&cyccg->slock);
	if (cyccg->idle_time > CYCCG_POLLING_IDLE_THRESHOLD)
		interval = msecs_to_jiffies(CYCCG_POLLING_TIMER_SLOW_INTERVAL);
	else
		interval = msecs_to_jiffies(CYCCG_POLLING_TIMER_INTERVAL);
	spin_unlock(&cyccg->slock);

	/* Reset the polling timer until it was stopped or IRQ actived. */
	spin_lock_irqsave(&cyccg->polling_timer_slock, flags);
	last_irq_time = cyccg->last_irq_time;
	spin_unlock_irqrestore(&cyccg->polling_timer_slock, flags);
	if (!last_irq_time)
		mod_delayed_work(system_wq, &cyccg->polling_timer, interval);
}

void cyccg_start_polling_timer(struct cyccg *cyccg, bool polling_timer_reset)
{
	unsigned long last_irq_time = 0;
	unsigned long flags;

	spin_lock_irqsave(&cyccg->polling_timer_slock, flags);
	if (polling_timer_reset)
		cyccg->last_irq_time = 0;
	else
		last_irq_time = cyccg->last_irq_time;
	spin_unlock_irqrestore(&cyccg->polling_timer_slock, flags);

	/* When the last_irq_time was updated, the IRQ must be actived. */
	if (last_irq_time)
		return;

	/* Always reset the idle time when the polling timer is restarted. */
	spin_lock(&cyccg->slock);
	cyccg->idle_time = 0;
	spin_unlock(&cyccg->slock);

	mod_delayed_work(system_wq, &cyccg->polling_timer,
			 msecs_to_jiffies(CYCCG_POLLING_TIMER_INTERVAL));
	cyccg_dbg("polling timer started\n");
}

static inline void cyccg_stop_polling_timer(struct cyccg *cyccg)
{
	cancel_delayed_work(&cyccg->polling_timer);
}

static inline int cyccg_polling_timer_idle_update(
		struct cyccg *cyccg, union hpi_intr_reg *intr)
{
	struct ccg_info *ccg_info = &cyccg->ccg_info;
	u8 intr_mask = 0x01;
	int ret = 0;

	if (ccg_info->hpi_ver != HPI_VERSION_1)
		intr_mask = ~(0xff << (ccg_info->num_port + 1));

	spin_lock(&cyccg->slock);
	if ((intr->val & intr_mask) == 0x00) {
		/* No interrupt asserted, idle state. */
		cyccg->idle_time += CYCCG_POLLING_TIMER_INTERVAL;
	} else {
		/* Event report is actived. */
		cyccg->idle_time = 0;
		ret = EBUSY;
	}
	spin_unlock(&cyccg->slock);
	return ret;
}

static irqreturn_t cyccg_irq_handler(int irq, void *dev_id)
{
	struct cyccg *cyccg = dev_id;

	cyccg_vdbg("---- IRQ asserted\n");
	/*
	 * Set the last_irq_time, so the polling timer can be stopped
	 * if the irq has been actived.
	 */
	spin_lock(&cyccg->polling_timer_slock);
	cyccg->last_irq_time = jiffies;
	spin_unlock(&cyccg->polling_timer_slock);
	cyccg_stop_polling_timer(cyccg);
	return IRQ_WAKE_THREAD;
}

static int cyccg_hpi_message_default_handler(struct hpi_device *hpidev,
				      struct hpi_msg *msg)
{
	struct cyccg *cyccg = hpidev->cyccg;
	struct hpi_swap_status *swap_status;
	enum hpi_swap_type swap_type;

	cyccg_vdbg("<<<< enter\n");
	if (IS_HPI_EVENT_MSG(msg->code) &&
			hpidev->dev_type >= HPI_DEV_TYPE_PORT) {
		/*
		 * When the port is disabled, there shall be no event message,
		 * so when any event message received, the port must be enabled.
		 */
		spin_lock(&hpidev->slock);
		hpidev->enabled = true;
		spin_unlock(&hpidev->slock);
	}

	switch (msg->code) {
	/* Responses, 0x00-0x7F */
	case HPI_PD_RESP_NO_RESPONSE:
		cyccg_dbg("HPI_PD_RESP_NO_RESPONSE\n");
		break;
	case HPI_PD_RESP_SUCCESS:
		cyccg_dbg("HPI_PD_RESP_SUCCESS\n");
		break;
	case HPI_PD_RESP_FLASH_DATA_AVAILABLE:
		cyccg_dbg("HPI_PD_RESP_FLASH_DATA_AVAILABLE\n");
		break;
	case HPI_PD_RESP_INVALID_COMMAND:
		cyccg_dbg("HPI_PD_RESP_INVALID_COMMAND\n");
		break;
	case HPI_PD_RESP_COLLISION_DETECTED:
		cyccg_dbg("HPI_PD_RESP_COLLISION_DETECTED\n");
		break;
	case HPI_PD_RESP_FLASH_UPDATE_FAILED:
		cyccg_dbg("HPI_PD_RESP_FLASH_UPDATE_FAILED\n");
		break;
	case HPI_PD_RESP_INVALID_FW:
		cyccg_dbg("HPI_PD_RESP_INVALID_FW\n");
		break;
	case HPI_PD_RESP_INVALID_ARGUMENTS:
		cyccg_dbg("HPI_PD_RESP_INVALID_ARGUMENTS\n");
		break;
	case HPI_PD_RESP_NOT_SUPPORTED:
		cyccg_dbg("HPI_PD_RESP_NOT_SUPPORTED\n");
		break;
	case HPI_PD_RESP_TRANSACTION_FAILED:
		cyccg_dbg("HPI_PD_RESP_TRANSACTION_FAILED\n");
		break;
	case HPI_PD_RESP_PD_COMMAND_FAILED:
		cyccg_dbg("HPI_PD_RESP_PD_COMMAND_FAILED\n");
		break;
	case HPI_PD_RESP_UNDEFINED_ERROR:
		cyccg_dbg("HPI_PD_RESP_UNDEFINED_ERROR\n");
		break;

	case HPI_PD_RESP_READ_PDO_DATA:
		cyccg_dbg("HPI_PD_RESP_READ_PDO_DATA\n");
		break;
	case HPI_PD_RESP_CMD_ABORTED:
		cyccg_dbg("HPI_PD_RESP_CMD_ABORTED\n");
		break;
	case HPI_PD_RESP_PORT_BUSY:
		cyccg_dbg("HPI_PD_RESP_PORT_BUSY\n");
		break;

	/* Device Specific Events, 0x80-0x81 */
	case HPI_PD_RESP_RESET_COMPLETE:
		cyccg_dbg("HPI_PD_RESP_RESET_COMPLETE\n");

		cyccg_queue_device_init_work(cyccg);
		break;
	case HPI_PD_RESP_MESSAGE_QUEUE_OVERFLOW:
		cyccg_dbg("HPI_PD_RESP_NO_RESPONSE\n");

		hpi_dump_pending_events(hpidev);
		return 0;

	/* Type C specific events, 0x82-0x85 */
	case HPI_PD_RESP_OVER_CURRENT_DETECTED:
		cyccg_dbg("HPI_PD_RESP_OVER_CURRENT_DETECTED\n");
		break;
	case HPI_PD_RESP_OVER_VOLTAGE_DETECTED:
		cyccg_dbg("HPI_PD_RESP_OVER_VOLTAGE_DETECTED\n");
		break;
	case HPI_PD_RESP_TYPE_C_CONNECTED:
		cyccg_dbg("HPI_PD_RESP_TYPE_C_CONNECTED\n");
		break;
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
		cyccg_dbg("HPI_PD_RESP_TYPE_C_DISCONNECTED\n");
		break;

	/* PD Specific events and asynchronous messages, 0x86-0x8F */
	case HPI_PD_RESP_PD_CONTRACT_ESTABLISHED:
		cyccg_dbg("HPI_PD_RESP_PD_CONTRACT_ESTABLISHED\n");

		/* Try to queue auto CC FW update if enabled. */
		cyccg_port_queue_cc_auto_fw_update(hpidev);
		break;
	case HPI_PD_RESP_SWAP_COMPLETE:
		cyccg_dbg("HPI_PD_RESP_SWAP_COMPLETE\n");
		swap_status = (struct hpi_swap_status *)msg->data;
		swap_type = (enum hpi_swap_type)swap_status->swap_type;
		switch (swap_type) {
		case HPI_DR_SWAP:
			cyccg_dbg("HPI_PD_RESP_DR_SWAP\n");
			break;
		case HPI_PR_SWAP:
			cyccg_dbg("HPI_PD_RESP_PR_SWAP\n");
			break;
		case HPI_VCONN_SWAP:
			cyccg_dbg("HPI_PD_RESP_VCON_SWAP\n");
			break;
		default:
			cyccg_dbg("unknown swap complete type = %u\n",
				swap_status->swap_type);
			break;
		}
		break;
	case HPI_PD_RESP_PS_RDY:
		cyccg_dbg("HPI_PD_RESP_PS_RDY\n");
		break;
	case HPI_PD_RESP_GOTOMIN:
		cyccg_dbg("HPI_PD_RESP_GOTOMIN\n");
		break;
	case HPI_PD_RESP_ACCEPT_MESSAGE:
		cyccg_dbg("HPI_PD_RESP_ACCEPT_MESSAGE\n");
		break;
	case HPI_PD_RESP_REJECT_MESSAGE:
		cyccg_dbg("HPI_PD_RESP_REJECT_MESSAGE\n");
		break;
	case HPI_PD_RESP_WAIT_MESSAGE:
		cyccg_dbg("HPI_PD_RESP_WAIT_MESSAGE\n");
		break;
	case HPI_PD_RESP_HARD_RESET:
		cyccg_dbg("HPI_PD_RESP_HARD_RESET\n");
		break;

	/* PD Data Message Specific Events, 0x90 */
	case HPI_PD_RESP_VDM_RECEIVED:
		cyccg_dbg("HPI_PD_RESP_VDM_RECEIVED\n");
		break;

	/* Capability Message Specific Events, 0x91-0x92*/
	case HPI_PD_RESP_SRC_CAP_RCVD:
		cyccg_dbg("HPI_PD_RESP_SRC_CAP_RCVD\n");
		break;
	case HPI_PD_RESP_SINK_CAP_RCVD:
		cyccg_dbg("HPI_PD_RESP_SINK_CAP_RCVD\n");
		break;

	/* DP and Alternate mode Specific Events, 0x93-0x99*/
	case HPI_PD_RESP_DP_ALTERNATE_MODE_ENTER:
		cyccg_dbg("HPI_PD_RESP_DP_ALTERNATE_MODE_ENTER\n");
		break;
	case HPI_PD_RESP_DP_STATUS_UPDATE:
		cyccg_dbg("HPI_PD_RESP_DP_STATUS_UPDATE\n");
		break;
	case HPI_PD_RESP_DP_SID_NOT_FOUND:
		cyccg_dbg("HPI_PD_RESP_DP_SID_NOT_FOUND\n");
		break;
	case HPI_PD_RESP_MULTIPLE_SVID_DISCOVERED:
		cyccg_dbg("HPI_PD_RESP_MULTIPLE_SVID_DISCOVERED\n");
		break;
	case HPI_PD_RESP_DP_FUNC_NOT_SUPPORTED_BY_CABLE:
		cyccg_dbg("HPI_PD_RESP_DP_FUNC_NOT_SUPPORTED_BY_CABLE\n");
		break;
	case HPI_PD_RESP_DP_PORT_CONFIG_NOT_SUPPORTED:
		cyccg_dbg("HPI_PD_RESP_DP_PORT_CONFIG_NOT_SUPPORTED\n");
		break;

	/* Resets and Error Scenario Events, 0x9A-0xA5*/
	case HPI_PD_HARD_RESET_SENT:
		cyccg_dbg("HPI_PD_HARD_RESET_SENT\n");
		break;
	case HPI_PD_SOFT_RESET_SENT:
		cyccg_dbg("HPI_PD_SOFT_RESET_SENT\n");
		break;
	case HPI_PD_CABLE_RESET_SENT:
		cyccg_dbg("HPI_PD_CABLE_RESET_SENT\n");
		break;
	case HPI_PD_SOURCE_DISBALED_STATE_ENTERED:
		cyccg_dbg("HPI_PD_SOURCE_DISBALED_STATE_ENTERED\n");
		break;
	case HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT:
		cyccg_dbg("HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT\n");
		break;
	case HPI_PD_NO_VDM_RESPONSE_RECEIVED:
		cyccg_dbg("HPI_PD_NO_VDM_RESPONSE_RECEIVED\n");
		break;
	case HPI_PD_UNEXPECTED_VOLTAGE_VBUS:
		cyccg_dbg("HPI_PD_UNEXPECTED_VOLTAGE_VBUS\n");
		break;
	case HPI_PD_TYPE_C_ERROR_RECOVERY:
		cyccg_dbg("HPI_PD_TYPE_C_ERROR_RECOVERY\n");
		break;

	/* EMCA Related Events, 0xA6-0xA7 */
	case HPI_PD_EMCA_DETECTED:
		cyccg_dbg("HPI_PD_EMCA_DETECTED\n");
		break;
	case HPI_PD_CABLE_DISCOVERY_FAILED:
		cyccg_dbg("HPI_PD_CABLE_DISCOVERY_FAILED\n");
		break;

	/* Miscellaneous Events */
	case HPI_PD_RP_CHANGE_DETECTED:
		cyccg_dbg("HPI_PD_RP_CHANGE_DETECTED\n");
		break;
	case HPI_PD_EC_VBUS_CONTROL:
		cyccg_dbg("HPI_PD_EC_VBUS_CONTROL\n");
		break;

	/* Billboard Related Events */
	case HPI_PD_BILLBOARD_CONNECT:
		cyccg_dbg("HPI_PD_BILLBOARD_CONNECT\n");
		break;
	case HPI_PD_SEND_VENDOR_DATA:
		cyccg_dbg("HPI_PD_SEND_VENDOR_DATA\n");
		break;

	/* Alternate Mode Related Events, HPIv2 only */
	case HPI_PD_ALTERNATE_MODE_EVENT:
		cyccg_dbg("HPI_PD_ALTERNATE_MODE_EVENT\n");
		/*
		 * Detail of this event data refer to definition of
		 * struct hpi_alternate_mode_event.
		 */
		break;
	case HPI_PD_ALTERNATE_MODE_HARDWARE_RESET:
		cyccg_dbg("HPI_PD_ALTERNATE_MODE_HARDWARE_RESET\n");
		/*
		 * Detail of this event data refer to definition of
		 * struct hpi_alternate_mode_hw_event.
		 */
		break;
	default:
		cyccg_dbg("warning: unknown HPI message code=0x%x, len=%zu\n",
			   msg->code, msg->len);
		break;
	}

	cyccg_dump("message data (code=0x%x, len=%zu):", msg->data, msg->len,
		msg->code, msg->len);
	hpi_clear_intr(hpidev);
	return 0;
}

static int cyccg_hpi_device_handler(struct hpi_device *hpidev)
{
	struct hpi_command *cmd;
	struct hpi_msg *msg;
	int err;

	cyccg_port_vdbg("<<<< enter\n", hpidev);
	if (!hpidev || !hpidev->cyccg ||
			hpidev->dev_type == HPI_DEV_TYPE_UNKNOWN) {
		cyccg_port_err("invalid hpidev instance\n", hpidev);
		return -EINVAL;
	}

	cmd = &hpidev->cmd;
	msg = &cmd->msg;
	if (!cmd->msg_buf.head || !cmd->msg_buf.size) {
		cyccg_port_err("invalid cmd->msg_buf\n", hpidev);
		err = -EINVAL;
		goto out;
	}

	err = hpi_message_read(hpidev, msg);
	if (err) {
		cyccg_port_err("failed to read HPI message, %d\n", hpidev, err);
		goto out;
	}

	if (hpidev->dev_type >= HPI_DEV_TYPE_PORT &&
			IS_HPI_PD_EVENT_CODE(msg->code)) {
		cyccg_port_cc_fw_update_state_reset(hpidev, msg);
		cyccg_update_pmic_usb_mode_status(hpidev, msg);
	}

	if (hpi_device_default_command_handler(hpidev, msg) ==
			HPI_MSG_RETURN_HANDLED) {
		cyccg_port_vdbg("command handled\n", hpidev);
		goto out;
	}

	/*
	 * TODO (Customer):
	 * More message process can be added intothe following function.
	 */
	cyccg_hpi_message_default_handler(hpidev, msg);

	if (hpidev->dev_type >= HPI_DEV_TYPE_PORT &&
			IS_HPI_PD_EVENT_CODE(msg->code))
		cyccg_run_usbc_pdport_callback(hpidev, msg);
	return 0;
out:
	hpi_clear_intr(hpidev);
	return err;
}

static irqreturn_t cyccg_irq_thread_handler(int irq, void *dev_id)
{
	struct cyccg *cyccg = dev_id;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	u16 port_num = cyccg->ccg_info.num_port;
	struct hpi_device *hpidev;
	union hpi_intr_reg intr;
	int i;
	int err;

	cyccg_vdbg("<<<< enter\n");

	spin_lock(&cyccg->slock);
	if (cyccg->irq_handler_state != CYCCG_WORK_STATE_NONE) {
		spin_unlock(&cyccg->slock);

		/*
		 * Polling timer or IRQ has caused the irq thrad handler
		 * been running, so retrun early.
		 */
		cyccg_vdbg("irq thread has been running, retrun early\n");
		return IRQ_HANDLED;
	}
	cyccg->irq_handler_state = CYCCG_WORK_STATE_RUNNING;
	spin_unlock(&cyccg->slock);

	do {
		err = hpi_read_intr_reg(cyccg, &intr);
		if (err) {
			cyccg_err("failed to read intr_reg\n");
			goto out;
		}

		cyccg_vdbg("intr.val = 0x%02x\n", intr.val);
		err = cyccg_polling_timer_idle_update(cyccg, &intr);
		if (!err)
			goto out;	/* No interrupt bit set, idle. */

		if (intr.dev_intr) {
			hpidev = &cyccg->hpi_dev;
			err = cyccg_hpi_device_handler(hpidev);
			cyccg_vdbg(">>> Device resp/event handled, %d\n", err);
			if (hpi_ver == HPI_VERSION_1)
				continue;
		}

		for (i = 0; i < port_num; i++) {
			hpidev = cyccg->ports[i];
			if (!IS_HPI_PORT_INTR_BIT_SET(intr.val, i) || !hpidev)
				continue;

			err = cyccg_hpi_device_handler(hpidev);
			cyccg_port_vdbg(">>> PD Port resp/event handled, %d\n",
				hpidev, err);
		}
	} while (true);

out:
	spin_lock(&cyccg->slock);
	cyccg->irq_handler_state = CYCCG_WORK_STATE_NONE;
	spin_unlock(&cyccg->slock);
	cyccg_vdbg(">>>> exit\n");
	return IRQ_HANDLED;
}

int cyccg_probe(struct device *dev, int irq, struct ccg_bus_operations *ops)
{
	struct cyccg *cyccg;
	struct hpi_device *port;
	int i;
	int err;

	cyccg_vdbg("<<<< enter\n");
	if (!dev || !ops || !ops->detect || !ops->read || !ops->write) {
		cyccg_err("invalid input parameters\n");
		return -EINVAL;
	}

	cyccg = devm_kzalloc(dev, sizeof(struct cyccg), GFP_KERNEL);
	if (!cyccg)
		return -ENOMEM;

	cyccg->dev = dev;
	cyccg->irq = irq;
	cyccg->bus_ops = ops;
	dev_set_drvdata(dev, cyccg);

	err = cyccg_dt_parse_and_get_pdata(cyccg);
	if (err) {
		cyccg_err("failed to parse and get pdata, %d\n", err);
		return err;
	}

	err = cyccg_gpio_init(cyccg);
	if (err) {
		cyccg_err("failed to initialize GPIO pins, %d\n", err);
		return err;
	}

	err = cyccg_device_detect(cyccg);
	if (err) {
		cyccg_err("failed to detect CCGx device, %d\n", err);
		return err;
	}

	err = cyccg_hpidev_allocate_init(cyccg);
	if (err) {
		cyccg_err("failed to allocate and init hpi_device, %d\n", err);
		return err;
	}

	/*
	 * Discard all pending events when the driver loaded, it's aimed to
	 * avoid any previous problem affect driver process.
	 */
	cyccg_dump_all_pending_events(cyccg);

	/* Start polling timer to support non-IRQ support device/FW App. */
	err = devm_request_threaded_irq(dev, irq,
					cyccg_irq_handler,
					cyccg_irq_thread_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					CYCCG_NAME, cyccg);
	if (err) {
		cyccg_err("failed to register irq thread handler, %d\n", err);
		return err;
	}

	cyccg_start_polling_timer(cyccg, false);
	err = cyccg_device_init(cyccg);
	if (err) {
		cyccg_warn("failed to init CCG device, running_mode=%u, %d\n",
			cyccg->ccg_state.running_mode, err);
		if (cyccg->ccg_state.running_mode !=
				CCG_FW_MODE_TYPE_BOOTLAODER)
			return err;

		/*
		 * Ignore device init error when CCG running in bootloader mode,
		 * so later be able to update the FW image for it to recovery
		 * the prossible error issue.
		 */
		cyccg_warn("running in bootlaoder mode, continue probe\n");
	}

	err = cyccg_sysfs_init(cyccg);
	if (err) {
		cyccg_err("failed to initialize sysfs interfaces, %d\n", err);
		return err;
	}

	/* Sync PMIC and USB mode status after boot detected. */
	for (i = 0; i < cyccg->ccg_info.num_port; i++) {
		port = cyccg->ports[i];
		if (port && port->dev_type >= HPI_DEV_TYPE_PORT)
			cyccg_update_pmic_usb_mode_status(port, NULL);
	}

	cyccg_queue_auto_fw_update(cyccg);

	#if defined(CONFIG_CUSTOMER_PD_TEST) && CONFIG_CUSTOMER_PD_TEST
	customer_pd_init();
	#endif

	spin_lock(&cyccg->slock);
	cyccg->probe_done = true;
	spin_unlock(&cyccg->slock);
	cyccg_dbg("probe_done = %s\n", cyccg->probe_done ? "true" : "false");

	device_set_wakeup_capable(cyccg->dev, true);
	return 0;
}
EXPORT_SYMBOL(cyccg_probe);

int cyccg_remove(struct device *dev)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	void (*event_callback)(char *port_name,
			u32 event_code, u8 *event_data, size_t size);
	struct hpi_device *port;
	int i;

	for (i = 0; i < HPI_MAX_PD_PORTS; i++) {
		port = cyccg->ports[i];
		if (port) {
			mutex_lock(&port->event_cb_mlock);
			event_callback = port->event_callback;
			mutex_unlock(&port->event_cb_mlock);
			if (event_callback) {
				event_callback(port->pdport_name_addr->name,
					PDPORT_DRV_REMOVED, NULL, 0);
			}
			cyccg_usbc_pdport_unregister(port);
		}
	}

	#if defined(CONFIG_CUSTOMER_PD_TEST) && CONFIG_CUSTOMER_PD_TEST
	customer_pd_uninit();
	#endif
	return 0;
}
EXPORT_SYMBOL(cyccg_remove);

int cyccg_suspend(struct device *dev)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);

	if (cyccg->irq)
		disable_irq(cyccg->irq);

	if (device_may_wakeup(cyccg->dev))
		device_set_wakeup_enable(cyccg->dev, true);

	return 0;
}
EXPORT_SYMBOL(cyccg_suspend);

int cyccg_resume(struct device *dev)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);

	if (device_may_wakeup(cyccg->dev))
		device_set_wakeup_enable(cyccg->dev, false);

	if (cyccg->irq)
		enable_irq(cyccg->irq);

	return 0;
}
EXPORT_SYMBOL(cyccg_resume);

const struct dev_pm_ops cyccg_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyccg_suspend, cyccg_resume)
};
EXPORT_SYMBOL(cyccg_pm_ops);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dudley Du <dudl@cypress.com>");
MODULE_DESCRIPTION("Cypress USB Type-C and PD Controller main module");