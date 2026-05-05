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

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "cyccg_core.h"

#define sysfs_err(fmt, ...)	ccg_err(fmt, ##__VA_ARGS__)
#define sysfs_warn(fmt, ...)	ccg_warn(fmt, ##__VA_ARGS__)
#define sysfs_info(fmt, ...)	ccg_info(fmt, ##__VA_ARGS__)
#define sysfs_dbg(fmt, ...)	ccg_dbg(fmt, ##__VA_ARGS__)
#define sysfs_vdbg(fmt, ...)	ccg_vdbg(fmt, ##__VA_ARGS__)
#define sysfs_dump(fmt, buf, size, ...)		\
	ccg_dump(fmt, buf, size, ##__VA_ARGS__)

#define sysfs_port_err(fmt, port, ...)	port_err(fmt, port, ##__VA_ARGS__)
#define sysfs_port_warn(fmt, port, ...)	port_warn(fmt, port, ##__VA_ARGS__)
#define sysfs_port_info(fmt, port, ...)	\
	port_info(fmt, port, ##__VA_ARGS__)
#define sysfs_port_dbg(fmt, port, ...)	port_dbg(fmt, port, ##__VA_ARGS__)
#define sysfs_port_vdbg(fmt, port, ...)	port_vdbg(fmt, port, ##__VA_ARGS__)
#define sysfs_port_dump(fmt, port, buf, size, ...)	\
	port_dump(fmt, port, buf, size, ##__VA_ARGS__)


#define to_hpi_device(kobj) container_of(kobj, struct hpi_device, kobj)

struct hpi_port_attribute {
	struct attribute attr;
	ssize_t (*show)(struct hpi_device *port,
			struct hpi_port_attribute *attr, char *buf);
	ssize_t (*store)(struct hpi_device *port,
			 struct hpi_port_attribute *attr,
			 const char *buf, size_t count);
};
#define to_hpi_port_attr(attr)		\
	container_of(attr, struct hpi_port_attribute, attr)
#define HPI_PORT_ATTR(_name, _mode, _show, _store)	\
	struct hpi_port_attribute _name##_attribute =	\
			__ATTR(_name, _mode, _show, _store)

static ssize_t hpi_port_attr_show(struct kobject *kobj, struct attribute *attr,
				  char *buf)
{
	struct hpi_port_attribute *attribute = to_hpi_port_attr(attr);
	struct hpi_device *port = to_hpi_device(kobj);

	if (!attribute->show)
		return -EIO;
	return attribute->show(port, attribute, buf);
}

static ssize_t hpi_port_attr_store(struct kobject *kobj, struct attribute *attr,
				   const char *buf, size_t len)
{
	struct hpi_port_attribute *attribute = to_hpi_port_attr(attr);
	struct hpi_device *port = to_hpi_device(kobj);

	if (!attribute->store)
		return -EIO;
	return attribute->store(port, attribute, buf, len);
}

static const struct sysfs_ops hpi_port_sysfs_ops = {
	.show = hpi_port_attr_show,
	.store = hpi_port_attr_store,
};

static void hpi_port_sysfs_release(struct kobject *kobj)
{
}

static ssize_t sysfs_name_addr_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	struct usbc_pdport_name_addr *port_name_addr = port->pdport_name_addr;
	int size;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (!port_name_addr) {
		sysfs_err("invalid port name addr pointer <NULL>\n");
		return -EINVAL;
	}

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	size = scnprintf(buf, PAGE_SIZE, "%s, <%s, %u, 0x%04x, %d>\n",
			port_name_addr->name ?: "UNKNOWN_PORT_NAME",
			port_name_addr->bustype == BUS_I2C ? "I2C" :
				port_name_addr->bustype == BUS_SPI ?
					"SPI" : "UNKNOWN_BUS_TYPE",
			port_name_addr->bus,
			port_name_addr->addr,
			port_name_addr->port_index);

	mutex_unlock(&cyccg->sysfs_mlock);
	return err ? err : size;

}
static HPI_PORT_ATTR(name_addr, S_IRUGO, sysfs_name_addr_show, NULL);

static ssize_t sysfs_index_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", port->idx);
}
static HPI_PORT_ATTR(index, S_IRUGO, sysfs_index_show, NULL);

static ssize_t sysfs_status_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	struct hpi_type_c_status type_c_status;
	struct hpi_pd_status pd_status;
	int size;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	memset(&pd_status, 0, sizeof(pd_status));
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = hpi_port_read_type_c_status(port, &type_c_status);
	if (err) {
		sysfs_port_err("failed to read port Type-c status, %d\n",
			port, err);
		goto out;
	}

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		sysfs_port_err("failed to read port PD status, %d\n",
			port, err);
		goto out;
	}

	size = scnprintf(buf, PAGE_SIZE, "Status of Port-%d, name: %s\n",
			port->idx, port->pdport_name_addr->name);
	size += scnprintf(buf + size, PAGE_SIZE - size,
		"Type_C_STATUS (0x%02x):\n", *(u8 *)&type_c_status);
	size += scnprintf(buf + size, PAGE_SIZE - size,
		"    type-c port connected: %s\n",
		type_c_status.type_c_connected ? "Yes" : "No");
	if (type_c_status.type_c_connected) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    cc_polarity: %s\n",
			type_c_status.cc_polarity ? "CC2" : "CC1");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    attached_dev_type: %s\n",
			hpi_attached_dev_type_to_string(
				type_c_status.attached_device_type));
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    ra_detected: %s\n",
			type_c_status.ra_detected ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    current_level: %s\n",
			hpi_type_c_current_level_to_string(
				type_c_status.current_level));
	}

	size += scnprintf(buf + size, PAGE_SIZE - size,
		"PD_STATUS (0x%08x):\n", *(u32 *)&pd_status);
	size += scnprintf(buf + size, PAGE_SIZE - size,
		"    contract_established: %s\n",
		pd_status.contract_established ? "Yes" : "No");
	if (pd_status.contract_established) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    current_data_role: %s\n",
			pd_status.current_data_role ? "DFP" : "UFP");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    current_power_role: %s\n",
			pd_status.current_power_role ? "Source" : "Sink");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    emca_present: %s\n",
			pd_status.emca_present ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    vconn_supplier: %s\n",
			pd_status.vconn_supplier ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    vconn_sourcing: %s\n",
			pd_status.vconn_sourcing ? "Yes" : "No");
	}

out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? err : size;
}
static HPI_PORT_ATTR(status, S_IRUGO, sysfs_status_show, NULL);

static ssize_t sysfs_enable_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	bool enabled;

	sysfs_vdbg("sysfs interface called\n");
	spin_lock(&port->slock);
	enabled = port->enabled;
	spin_unlock(&port->slock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!enabled);
}

static ssize_t sysfs_enable_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	char on_off_str[10];
	bool enable_port;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (count > sizeof(on_off_str)) {
		sysfs_err("invalid input parameters, too long\n");
		return -EINVAL;
	}

	memset(on_off_str, 0, sizeof(on_off_str));
	memcpy(on_off_str, buf, count);
	if (on_off_str[count - 1] == '\n')
		on_off_str[count - 1] = '\0';

	if (!strcasecmp(on_off_str, "1") || !strcasecmp(on_off_str, "on") ||
			!strcasecmp(on_off_str, "enable")) {
		enable_port = true;
	} else if (!strcasecmp(on_off_str, "0") ||
			!strcasecmp(on_off_str, "off") ||
			!strcasecmp(on_off_str, "disable")) {
		enable_port = false;
	} else {
		sysfs_err("unknown port enable/disable command=%s\n",
			on_off_str);
		return -EINVAL;
	}

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	sysfs_port_vdbg("%s port\n", port, enable_port ? "enable" : "disable");
	if (enable_port) {
		err = hpi_port_enable_sync(port);
		if (!err) {
			if (cyccg_port_init(port))
				sysfs_port_dbg("failed to init port\n", port);
		}
	} else {
		err = hpi_port_disable_sync(port);
	}
	if (err)
		sysfs_port_err("failed to %s port, %d\n", port,
			enable_port ? "enable" : "disable", err);

	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		sysfs_enable_show, sysfs_enable_store);

static ssize_t sysfs_current_pdo_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{

	struct cyccg *cyccg = port->cyccg;
	struct hpi_pd_status pd_status;
	union power_data_object *pdo;
	enum power_supply_type power_type;
	u32 current_pdo;
	bool is_source;
	u32 val;
	int size;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	memset(&pd_status, 0, sizeof(pd_status));
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		sysfs_port_err("failed to read PD status, %d\n",
			port, err);
		goto out;
	}

	err = hpi_port_read_current_pdo(port, &current_pdo);
	if (err) {
		sysfs_port_err("failed to read current PDO, %d\n",
			port, err);
		goto out;
	}

	pdo = (union power_data_object *)&current_pdo;
	is_source = pd_status.current_power_role ? true : false;
	size = scnprintf(buf, PAGE_SIZE,
			"Current %s PDO(=0x%08x) of %s (Port-%d):\n",
			is_source ? "Source" : "Sink", current_pdo,
			port->pdport_name_addr->name, port->idx);
	power_type =
		(enum power_supply_type)pdo->source_battery.power_supply_type;
	if (power_type == POWER_TYPE_FIXED_SUPPLY && is_source) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Fixed supply (Vmin = Vmax)\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Dual-Role Power: %s\n",
			pdo->source_fixed_supply.dual_role_power ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    USB Suspend Supported: %s\n",
			pdo->source_fixed_supply.usb_suspend_supported ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Externally Powered: %s\n",
			pdo->source_fixed_supply.externally_powered ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    USB Communications Capable: %s\n",
			pdo->source_fixed_supply.usb_communications_capable ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Dual-Role Data: %s\n",
			pdo->source_fixed_supply.dual_role_data ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Unchunked Extended Messages Supported: %s\n",
		pdo->source_fixed_supply.unchunked_extended_messages_supported ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Peak Current: %u\n",
			pdo->source_fixed_supply.peak_current);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Voltage in 50mV units: %u\n",
			pdo->source_fixed_supply.voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Current in 10mA units: %u\n",
			pdo->source_fixed_supply.maximum_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_BATTERY && is_source) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Battery\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Voltage in 50mV units: %u\n",
			pdo->source_battery.maximum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Minimum Voltage in 50mV units: %u\n",
			pdo->source_battery.minimum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Allowable Power in 250mV units: %u\n",
		pdo->source_battery.maximum_allowable_power_in_250mW_units);
	} else if (power_type == POWER_TYPE_VARIABLE_SUPPLY && is_source) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Variable Supply (non-Battery)\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Voltage in 50mV units: %u\n",
		pdo->source_variable_supply.maximum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Minimum Voltage in 50mV units: %u\n",
		pdo->source_variable_supply.minimum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Current in 10mA units: %u\n",
		pdo->source_variable_supply.maximum_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_FIXED_SUPPLY && !is_source) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Fixed supply\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Dual-Role Power: %s\n",
			pdo->sink_fixed_supply.dual_role_power ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Higher Capability: %s\n",
			pdo->sink_fixed_supply.higher_capability ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Externally Powered: %s\n",
			pdo->sink_fixed_supply.externally_powered ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    USB Communications Capable: %s\n",
			pdo->sink_fixed_supply.usb_communications_capable ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Dual-Role Data: %s\n",
			pdo->sink_fixed_supply.dual_role_data ? "Yes" : "No");
		val = pdo->sink_fixed_supply.fast_role_swap_required_current;
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Fast Role Swap required USB Type-C Current: %s\n",
			(val == 0) ? "Fast Swap not supported (default)" :
				((val == 1) ? "Default USB Power" :
				((val == 2) ? "1.5A @ 5V" : "3.0A @ 5V")));
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Voltage in 50mV units: %u\n",
			pdo->sink_fixed_supply.voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Operational Current in 10mA units: %u\n",
		pdo->sink_fixed_supply.operational_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_BATTERY && !is_source) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Battery\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Voltage in 50mV units: %u\n",
			pdo->sink_battery.maximum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Minimum Voltage in 50mV units: %u\n",
			pdo->sink_battery.minimum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Operational Power in 250mW units: %u\n",
			pdo->sink_battery.operational_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_VARIABLE_SUPPLY && !is_source) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Variable Supply (non-Battery)\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Voltage in 50mV units: %u\n",
		pdo->sink_variable_supply.maximum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Minimum Voltage in 50mV units: %u\n",
		pdo->sink_variable_supply.minimum_voltage_in_50mV_units);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Operational Current in 10mA units: %u\n",
		pdo->sink_variable_supply.operational_power_in_250mW_units);
	} else {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Reserved\n");
	}
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? err : size;

}
static HPI_PORT_ATTR(current_pdo, S_IRUGO, sysfs_current_pdo_show, NULL);

static ssize_t sysfs_current_rdo_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	union power_data_object *pdo;
	union request_data_object *rdo;
	enum power_supply_type power_type;
	bool is_giveback_flag_set;
	u32 current_pdo;
	u32 current_rdo;
	int size;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = hpi_port_read_current_pdo(port, &current_pdo);
	if (err) {
		sysfs_port_err("failed to read current PDO, %d\n", port, err);
		goto out;
	}

	err = hpi_port_read_current_rdo(port, &current_rdo);
	if (err) {
		sysfs_port_err("failed to read current RDO, %d\n", port, err);
		goto out;
	}

	pdo = (union power_data_object *)&current_pdo;
	rdo = (union request_data_object *)&current_rdo;
	power_type =
		(enum power_supply_type)pdo->source_battery.power_supply_type;
	size = scnprintf(buf, PAGE_SIZE,
			"Current RDO(=0x%08x) of %s (Port-%d):\n",
			current_rdo, port->pdport_name_addr->name, port->idx);
	if (power_type == POWER_TYPE_FIXED_SUPPLY ||
			power_type == POWER_TYPE_VARIABLE_SUPPLY) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Fixed and Variable Request Data Object\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Object position: %u\n",
			rdo->fixed_variable.object_position);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    GiveBack Flag: %u\n",
			rdo->fixed_variable.giveback_flag);
		is_giveback_flag_set =
			rdo->fixed_variable.giveback_flag ? true : false;
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Capability Mismatch: %s\n",
			rdo->fixed_variable.capability_mismatch ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    USB Communications Capable: %s\n",
			rdo->fixed_variable.usb_communications_capable ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    No USB Suspend: %s\n",
			rdo->fixed_variable.no_usb_suspend ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Unchunked Extended Messages Support: %s\n",
		rdo->fixed_variable.unchunked_extended_messages_supported
				? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Operating Current in 10mA units: %u\n",
			rdo->fixed_variable.operating_current_in_10mA_units);
		if (is_giveback_flag_set)
			size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Minimum Operating Current in 10mA units: %u\n",
				(rdo->fixed_variable_giveback.
					minimum_operating_current_10mA_units));
		else
			size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Operating Current in 10mA units: %u\n",
				(rdo->fixed_variable.
					maximum_operating_current_10mA_units));
	} else if (power_type == POWER_TYPE_BATTERY) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Battery Request Data Object\n");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Object position: %u\n",
			rdo->battery.object_position);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    GiveBack Flag: %u\n", rdo->battery.giveback_flag);
		is_giveback_flag_set =
			rdo->battery.giveback_flag ? true : false;
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Capability Mismatch: %s\n",
			rdo->battery.capability_mismatch ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    USB Communications Capable: %s\n",
			rdo->battery.usb_communications_capable ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    No USB Suspend: %s\n",
			rdo->battery.no_usb_suspend ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Unchunked Extended Messages Support: %s\n",
			rdo->battery.unchunked_extended_messages_supported ?
				"Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Operating Power in 250mW units: %u\n",
			rdo->battery.operating_power_in_250mW_units);
		if (is_giveback_flag_set)
			size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Minimum Operating Power in 250mA units: %u\n",
				(rdo->battery_giveback.
				minimum_operating_power_in_250mW_units));
		else
			size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Operating Power in 250mA units: %u\n",
			(rdo->battery.maximum_operating_power_in_250mW_units));
	} else {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Reserved\n");
	}
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? err : size;
}
static HPI_PORT_ATTR(current_rdo, S_IRUGO, sysfs_current_rdo_show, NULL);

static inline char *get_cable_latency_string(u32 latency)
{
	switch (latency) {
	case 1:
		return "<10ns (~1m)";
	case 2:
		return "10ns to 20ns (~2m)";
	case 3:
		return "20ns to 30ns (~3m)";
	case 4:
		return "30ns to 40ns (~4m)";
	case 5:
		return "40ns to 50ns (~5m)";
	case 6:
		return "50ns to 60ns (~6m)";
	case 7:
		return "60ns to 70ns (~7m)";
	case 8:
		return "> 70ns (>~7m)";
	default:
		break;
	}

	return "Reserved, shall not be used";
}

static inline char *get_cable_termination_type_string(u32 termination_type)
{
	switch (termination_type) {
	case 0:
		return "VCONN not required";
	case 1:
		return "VCONN required";
	case 2:
		return "One end Active, one end passive, VCONN required";
	case 3:
		return "Both ends Active, VCONN required";
	default:
		break;
	}

	return "Reserved, should not used";
}

static ssize_t sysfs_current_cable_vdo_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	union vdm_avtive_cable_vdo *cable_vdo;
	u32 current_cable_vdo;
	int size;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = hpi_port_read_current_cable_vdo(port, &current_cable_vdo);
	if (err) {
		sysfs_port_err("failed to read current PDO, %d\n", port, err);
		goto out;
	}

	cable_vdo = (union vdm_avtive_cable_vdo *)&current_cable_vdo;
	size = scnprintf(buf, PAGE_SIZE,
			"Current Cable VDO(=0x%08x) of %s (Port-%d):\n",
			current_cable_vdo, port->pdport_name_addr->name,
			port->idx);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    HW Version: %u\n", cable_vdo->hw_version);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    FW Version: %u\n", cable_vdo->fw_version);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    VDO Version: %u\n", cable_vdo->vdo_version + 1);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Plug to: %s\n",
			cable_vdo->plug_to == 2 ? "USB Type-C" :
			(cable_vdo->plug_to == 3 ? "Captive" :
				"Reserved, shall not be used"));
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Cable Latency: %s\n",
			get_cable_latency_string(cable_vdo->latency));
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Cable Termination Type: %s\n",
			get_cable_termination_type_string(
				cable_vdo->termination_type));
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    Maximum Cable VBus Voltage: %uV\n",
			cable_vdo->maximun_vbus_voltage * 10 + 20);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    VBus Current Handling Capability: %s\n",
			cable_vdo->vbus_current == 1 ? "3A" :
			(cable_vdo->vbus_current == 2 ? "5A" : "Reserved"));
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    VBus Through Cable: %s\n",
			cable_vdo->vbus_through_cable ? "Yes" : "No");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    SOP'' Controller Present: %s\n",
			cable_vdo->sop_dprim_present ? "Yes" : "No");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"    USB SuperSpeed Signaling Support: %s\n",
			cable_vdo->ss_signal_type == 0 ? "USB 2.0 Only" :
			(cable_vdo->ss_signal_type == 1 ? "[USB 3.1] Gen1" :
			(cable_vdo->ss_signal_type == 2 ?
				"[USB 3.1] Gen1 and Gen2" : "Reserved")));
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? err : size;
}
static HPI_PORT_ATTR(current_cable_vdo, S_IRUGO,
		sysfs_current_cable_vdo_show, NULL);

static ssize_t sysfs_data_role_swap_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_data_role_swap_sync(port);
	if (err)
		sysfs_port_err("data role swap failed, %d\n", port, err);

	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(data_role_swap, S_IWUSR | S_IWGRP,
		NULL, sysfs_data_role_swap_store);

static ssize_t sysfs_power_role_swap_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_power_role_swap_sync(port);
	if (err)
		sysfs_port_err("power role swap failed, %d\n", port, err);

	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(power_role_swap, S_IWUSR | S_IWGRP,
		NULL, sysfs_power_role_swap_store);

static ssize_t sysfs_vconn_swap_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_vconn_role_swap_sync(port);
	if (err)
		sysfs_port_err("Vconn swap failed, %d\n", port, err);

	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(vconn_swap, S_IWUSR | S_IWGRP,
		NULL, sysfs_vconn_swap_store);

static ssize_t sysfs_vconn_switch_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	struct hpi_pd_status pd_status;
	int size;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	memset(&pd_status, 0, sizeof(pd_status));
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		sysfs_port_err("read PD status failed, %d\n", port, err);
		goto out;
	}

	size = scnprintf(buf, PAGE_SIZE, "%u\n", pd_status.vconn_sourcing);

out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : size;
}

static ssize_t sysfs_vconn_switch_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	char on_off_str[10];
	bool sourcing_on;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (count > sizeof(on_off_str)) {
		sysfs_err("invalid input parameters, too long\n");
		return -EINVAL;
	}

	memset(on_off_str, 0, sizeof(on_off_str));
	memcpy(on_off_str, buf, count);
	if (on_off_str[count - 1] == '\n')
		on_off_str[count - 1] = '\0';

	if (!strcasecmp(on_off_str, "1") || !strcasecmp(on_off_str, "on") ||
			!strcasecmp(on_off_str, "enable")) {
		sourcing_on = true;
	} else if (!strcasecmp(on_off_str, "0") ||
			!strcasecmp(on_off_str, "off") ||
			!strcasecmp(on_off_str, "disable")) {
		sourcing_on = false;
	} else {
		sysfs_err("unknown VCONN sourcing enable/disable command=%s\n",
			on_off_str);
		return -EINVAL;
	}

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_switch_vconn_sync(port, sourcing_on);
	if (err)
		sysfs_port_err("turn Vconn sourcing %s failed, %d\n",
			port, sourcing_on ? "on" : "off", err);

	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(vconn_switch, S_IRUGO | S_IWUSR | S_IWGRP,
	       sysfs_vconn_switch_show, sysfs_vconn_switch_store);

static ssize_t sysfs_sop_type_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	int size;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	size = scnprintf(buf, PAGE_SIZE, "%s\n",
			hpi_vdm_sop_type_to_string(port->sysfs_sop_type));

	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? err : size;
}

static ssize_t sysfs_sop_type_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	enum vdm_sop_type sop_type;
	size_t len;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (buf[count - 1] == '\n')
		len = count - 1;
	else
		len = count;

	if (len == 1) {
		sop_type = (enum vdm_sop_type)((u8)buf[0] - (u8)'0');
		switch (sop_type) {
		case VDM_SOP_TYPE_SOP:
		case VDM_SOP_TYPE_SOP_PRIME:
		case VDM_SOP_TYPE_SOP_DPRIME:
			break;
		default:
			sysfs_err("invalid input sop_type=%u\n", (u8)sop_type);
			return -EINVAL;
		}
	} else {
		if (len == strlen("SOP") && strncasecmp(buf, "SOP", len)) {
			sop_type = VDM_SOP_TYPE_SOP_DPRIME;
		} else if (len == strlen("SOP_PRIME") &&
				strncasecmp(buf, "SOP_PRIME", len)) {
			sop_type = VDM_SOP_TYPE_SOP_PRIME;
		} else if (len == strlen("SOP_DPRIME") &&
				strncasecmp(buf, "SOP_DPRIME", len)) {
			sop_type = VDM_SOP_TYPE_SOP;
		} else {
			sysfs_err("unknown sop_type string=%s\n", buf);
			return -EINVAL;
		}
	}

	sysfs_vdbg("set sop_type=%s\n",
			hpi_vdm_sop_type_to_string(port->sysfs_sop_type));

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	port->sysfs_sop_type = sop_type;

	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? err : count;
}
static HPI_PORT_ATTR(sop_type, S_IRUGO | S_IWUSR | S_IWGRP,
	       sysfs_sop_type_show, sysfs_sop_type_store);

static ssize_t sysfs_send_vdm_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	char **data_array;
	int data_array_count;
	unsigned long byte_val;
	u8 vdm_data[VDM_MAX_DATA_SIZE];
	int i;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	sysfs_vdbg("input buf[%zu] = %s\n", count, buf);
	data_array = argv_split(GFP_KERNEL, buf, &data_array_count);
	if (!data_array || !data_array_count) {
		sysfs_err("invalid input VDM data\n");
		return -EINVAL;
	}

	for (i = 0; i < data_array_count; i++) {
		if (kstrtoul(data_array[i], 16, &byte_val) || byte_val > 255) {
			sysfs_err("invalid VDM data value\n");
			err = -EINVAL;
			goto out;
		}

		vdm_data[i] = (u8)byte_val;
	}

	sysfs_dump("Send VDM Data:\n", &vdm_data[0], data_array_count);

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		err = -EAGAIN;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto err;
	}

	err = hpi_port_send_vdm_data_generic(port,
		port->sysfs_sop_type, HPI_CMD_FLAG_RESP_EVENT,
		vdm_data, data_array_count,
		NULL, NULL, HPI_ASYNC, NULL, NULL, NULL, 0);
	if (err)
		sysfs_port_err("failed to send VDM data, %d\n", port, err);

	cyccg_set_to_busying_state(port, false);
err:
	mutex_unlock(&cyccg->sysfs_mlock);
out:
	argv_free(data_array);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(send_vdm, S_IWUSR | S_IWGRP, NULL, sysfs_send_vdm_store);

static ssize_t sysfs_get_source_cap_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	u8 data[USBPD_MAX_DATA_OBJS * USBPD_DATA_OBJ_SIZE];
	size_t data_size = sizeof(data);
	size_t size = 0;
	int i = 0;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_get_partner_source_capabilities_sync(port,
			data, &data_size);
	if (err || !data_size) {
		sysfs_port_err("get port partner SOURCE_CAP failed, %d\n",
			port, err);
		if (!err) {
			err = -EINVAL;
			sysfs_port_err("invalid received data_size=%zu\n",
				port, data_size);
		}
		goto busy;
	}

	size = scnprintf(buf, PAGE_SIZE, "%02x", data[0]);
	while (++i < data_size)
		size += scnprintf(buf + size, PAGE_SIZE - size,
				" %02x", data[i]);
	size += scnprintf(buf + size, PAGE_SIZE - size, "\n");


busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EINVAL : err) : size;
}
static HPI_PORT_ATTR(get_source_cap, S_IRUGO, sysfs_get_source_cap_show, NULL);

static ssize_t sysfs_get_sink_cap_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	u8 data[USBPD_MAX_DATA_OBJS * USBPD_DATA_OBJ_SIZE];
	size_t data_size = sizeof(data);
	size_t size = 0;
	int i = 0;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = hpi_port_get_partner_sink_capabilities_sync(port,
			data, &data_size);
	if (err || !data_size) {
		sysfs_port_err("get port partner SINK_CAP failed, %d\n",
			port, err);
		if (!err) {
			err = -EINVAL;
			sysfs_port_err("invalid received data_size=%zu\n",
				port, data_size);
		}
		goto busy;
	}

	size = scnprintf(buf, PAGE_SIZE, "%02x", data[0]);
	while (++i < data_size)
		size += scnprintf(buf + size, PAGE_SIZE - size,
				" %02x", data[i]);
	size += scnprintf(buf + size, PAGE_SIZE - size, "\n");

busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EINVAL : err) : size;
}
static HPI_PORT_ATTR(get_sink_cap, S_IRUGO, sysfs_get_sink_cap_show, NULL);

static ssize_t sysfs_cc_silicon_id_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	u8 cy_mode_obj_pos;
	u32 silicon_id;
	u8 uuid[HPI_FW_IMAGE_UUID_SIZE];
	enum ccg_version ccg_ver;
	int size = 0;
	int i;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (port->sysfs_sop_type == VDM_SOP_TYPE_SOP_DPRIME) {
		sysfs_err("SOP_DRPIME is not supported\n");
		err = -ENOTSUPP;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_cc_enter_dfp_mode(port);
	if (err) {
		sysfs_port_err("failed to enter DFP mode, %d\n", port, err);
		goto busy;
	}

	err = cyccg_update_cc_enter_cy_mode(port, port->sysfs_sop_type,
					    &cy_mode_obj_pos);
	if (err) {
		sysfs_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		goto busy;
	}

	memset(uuid, 0, sizeof(uuid));
	err = cyccg_port_cc_get_silicon_id(port,
			cyccg->sysfs_port_sop_type, &silicon_id,
			uuid, HPI_FW_IMAGE_UUID_SIZE);
	if (err) {
		sysfs_port_err("failed to read silicon, %d\n", port, err);
		goto err;
	}
	ccg_ver = ccg_silicon_id_to_ccg_version((u16)silicon_id);

	size = scnprintf(buf, PAGE_SIZE, "Silicon ID: 0x%04x\n", silicon_id);
	if (ccg_ver >= CCG3) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "UUID(0x):");
		for (i = 0; i < HPI_FW_IMAGE_UUID_SIZE; i++) {
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"%02X", uuid[i]);
		}
		size += scnprintf(buf + size, PAGE_SIZE - size, "\n");
	}

err:
	cyccg_update_cc_reset_and_init(port,
		cyccg->sysfs_port_sop_type, cy_mode_obj_pos, false);
busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : size;
}
static HPI_PORT_ATTR(cc_silicon_id, S_IRUGO, sysfs_cc_silicon_id_show, NULL);

static ssize_t sysfs_cc_fw_version_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	struct hpi_ccg_fw_version ccg_fw_vers;
	u8 cy_mode_obj_pos;
	int size = 0;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (port->sysfs_sop_type == VDM_SOP_TYPE_SOP_DPRIME) {
		sysfs_err("SOP_DRPIME is not supported\n");
		err = -ENOTSUPP;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_cc_enter_dfp_mode(port);
	if (err) {
		sysfs_port_err("failed to enter DFP mode, %d\n", port, err);
		goto busy;
	}

	err = cyccg_update_cc_enter_cy_mode(port, cyccg->sysfs_port_sop_type,
					    &cy_mode_obj_pos);
	if (err) {
		sysfs_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		goto busy;
	}

	err = cyccg_port_cc_get_device_version(port,
			cyccg->sysfs_port_sop_type, &ccg_fw_vers);
	if (err) {
		sysfs_port_err("failed to read device version, %d\n",
			port, err);
		goto err;
	}

	size = scnprintf(buf, PAGE_SIZE,
			"bootloader base version: %u.%u.%u.%u\n",
			ccg_fw_vers.btldr.base.major,
			ccg_fw_vers.btldr.base.minor,
			ccg_fw_vers.btldr.base.patch_ver,
			ccg_fw_vers.btldr.base.build_number);

	size += scnprintf(buf + size, PAGE_SIZE - size,
			"bootloader app version: %u.%u,%u,%c%c\n",
			ccg_fw_vers.btldr.app.major,
			ccg_fw_vers.btldr.app.minor,
			ccg_fw_vers.btldr.app.external_circuit_ver,
			ccg_fw_vers.btldr.app.name[0],
			ccg_fw_vers.btldr.app.name[1]);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"APP/FW image-1 base version: %u.%u.%u.%u\n",
			ccg_fw_vers.fw1_app.base.major,
			ccg_fw_vers.fw1_app.base.minor,
			ccg_fw_vers.fw1_app.base.patch_ver,
			ccg_fw_vers.fw1_app.base.build_number);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"APP/FW image-1 app version: %u.%u,%u,%c%c\n",
			ccg_fw_vers.fw1_app.app.major,
			ccg_fw_vers.fw1_app.app.minor,
			ccg_fw_vers.fw1_app.app.external_circuit_ver,
			ccg_fw_vers.fw1_app.app.name[0],
			ccg_fw_vers.fw1_app.app.name[1]);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"FW image-2 base version: %u.%u.%u.%u\n",
			ccg_fw_vers.fw2_app.base.major,
			ccg_fw_vers.fw2_app.base.minor,
			ccg_fw_vers.fw2_app.base.patch_ver,
			ccg_fw_vers.fw2_app.base.build_number);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"FW image-2 app version: %u.%u,%u,%c%c\n",
			ccg_fw_vers.fw2_app.app.major,
			ccg_fw_vers.fw2_app.app.minor,
			ccg_fw_vers.fw2_app.app.external_circuit_ver,
			ccg_fw_vers.fw2_app.app.name[0],
			ccg_fw_vers.fw2_app.app.name[1]);
err:
	cyccg_update_cc_reset_and_init(port,
		cyccg->sysfs_port_sop_type, cy_mode_obj_pos, false);
busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : size;
}
static HPI_PORT_ATTR(cc_fw_version, S_IRUGO, sysfs_cc_fw_version_show, NULL);

static ssize_t sysfs_cc_running_mode_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	enum ccg_fw_mode_type running_mode;
	u8 cy_mode_obj_pos;
	u32 silicon_id;
	enum ccg_version ccg_ver;
	int size = 0;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (port->sysfs_sop_type == VDM_SOP_TYPE_SOP_DPRIME) {
		sysfs_err("SOP_DRPIME is not supported\n");
		err = -ENOTSUPP;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_cc_enter_dfp_mode(port);
	if (err) {
		sysfs_port_err("failed to enter DFP mode, %d\n", port, err);
		goto busy;
	}

	err = cyccg_update_cc_enter_cy_mode(port, cyccg->sysfs_port_sop_type,
					    &cy_mode_obj_pos);
	if (err) {
		sysfs_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		goto busy;
	}

	err = cyccg_port_cc_get_silicon_id(port,
				cyccg->sysfs_port_sop_type, &silicon_id,
				NULL, 0);
	if (err) {
		sysfs_port_err("failed to read silicon, %d\n", port, err);
		goto err;
	}
	ccg_ver = ccg_silicon_id_to_ccg_version((u16)silicon_id);

	err = cyccg_port_cc_get_device_mode(port,
			cyccg->sysfs_port_sop_type, &running_mode, NULL);
	if (err) {
		sysfs_port_err("failed to read device mode, %d\n", port, err);
		goto err;
	}

	size = scnprintf(buf, PAGE_SIZE, "%u (%s)\n", running_mode,
			ccg_fw_mode_type_to_string(running_mode, ccg_ver));
err:
	cyccg_update_cc_reset_and_init(port,
		cyccg->sysfs_port_sop_type, cy_mode_obj_pos, false);
busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : size;
}
static HPI_PORT_ATTR(cc_running_mode, S_IRUGO,
		sysfs_cc_running_mode_show, NULL);

static ssize_t sysfs_cc_boot_reason_show(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		char *buf)
{
	struct cyccg *cyccg = port->cyccg;
	enum ccg_fw_mode_type running_mode;
	struct hpi_boot_mode_reason reason;
	u8 cy_mode_obj_pos;
	int size = 0;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (port->sysfs_sop_type == VDM_SOP_TYPE_SOP_DPRIME) {
		sysfs_err("SOP_DRPIME is not supported\n");
		err = -ENOTSUPP;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_cc_enter_dfp_mode(port);
	if (err) {
		sysfs_port_err("failed to enter DFP mode, %d\n", port, err);
		goto busy;
	}

	err = cyccg_update_cc_enter_cy_mode(port, cyccg->sysfs_port_sop_type,
					    &cy_mode_obj_pos);
	if (err) {
		sysfs_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		goto busy;
	}

	err = cyccg_port_cc_get_device_mode(port,
			cyccg->sysfs_port_sop_type, &running_mode, NULL);
	if (err) {
		sysfs_port_err("failed to read device mode, %d\n", port, err);
		goto err;
	}

	if (running_mode != CCG_FW_MODE_TYPE_BOOTLAODER) {
		size = scnprintf(buf, PAGE_SIZE,
			"not running in bootloader mode, running_mode=%d\n",
			(int)running_mode);
		err = 0;
		goto err;
	}

	err = cyccg_port_cc_get_boot_mode_reason(port,
			cyccg->sysfs_port_sop_type, &reason);
	if (err) {
		sysfs_port_err("failed to read boot mode reason, %d\n",
			port, err);
		goto err;
	}

	size = scnprintf(buf, PAGE_SIZE, "Boot mode requested by EC = %s\n",
			reason.boot_mode_request ? "true" : "false");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"Config table is valid = %s\n",
			reason.config_table_status ? "true" : "false");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"FW App 1 is valid = %s\n",
			reason.fw_app_1_status ? "true" : "false");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"FW App 2 is valid = %s\n",
			reason.fw_app_2_status ? "true" : "false");

err:
	cyccg_update_cc_reset_and_init(port,
		cyccg->sysfs_port_sop_type, cy_mode_obj_pos, false);
busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : size;
}
static HPI_PORT_ATTR(cc_boot_reason, S_IRUGO, sysfs_cc_boot_reason_show, NULL);

static ssize_t sysfs_cc_reset_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	u8 cy_mode_obj_pos;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (port->sysfs_sop_type == VDM_SOP_TYPE_SOP_DPRIME) {
		sysfs_err("SOP_DRPIME is not supported\n");
		err = -ENOTSUPP;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_cc_enter_dfp_mode(port);
	if (err) {
		sysfs_port_err("failed to enter DFP mode, %d\n", port, err);
		goto busy;
	}

	err = cyccg_update_cc_enter_cy_mode(port, cyccg->sysfs_port_sop_type,
					    &cy_mode_obj_pos);
	if (err) {
		sysfs_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		goto busy;
	}

	err = cyccg_update_cc_reset_and_init(port,
			cyccg->sysfs_port_sop_type, cy_mode_obj_pos, true);
	if (err) {
		sysfs_port_err("failed to reset and init, %d\n", port, err);
		goto busy;
	}
busy:
	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(cc_reset, S_IWUSR | S_IWGRP, NULL, sysfs_cc_reset_store);

static ssize_t sysfs_cc_update_fw_store(
		struct hpi_device *port, struct hpi_port_attribute *attr,
		const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (port->sysfs_sop_type == VDM_SOP_TYPE_SOP_DPRIME) {
		sysfs_err("SOP_DRPIME is not supported\n");
		err = -ENOTSUPP;
		goto out;
	}

	if (cyccg_busying_check_and_set(port)) {
		sysfs_port_err("CC port or CCG is busying\n", port);
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_cc_do_port_fw_update(port,
			cyccg->sysfs_port_ccg_type, cyccg->sysfs_port_sop_type,
			false, buf, count);
	if (err)
		sysfs_port_err("failed to update %s FW, %d\n", port,
			ccg_type_to_string(cyccg->sysfs_port_ccg_type), err);

	cyccg_set_to_busying_state(port, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static HPI_PORT_ATTR(cc_update_fw, S_IWUSR | S_IWGRP,
		NULL, sysfs_cc_update_fw_store);

static struct attribute *hpi_port_sysfs_default_attrs[] = {
	&name_addr_attribute.attr,
	&index_attribute.attr,
	&status_attribute.attr,
	&enable_attribute.attr,
	&current_pdo_attribute.attr,
	&current_rdo_attribute.attr,
	&current_cable_vdo_attribute.attr,
	&data_role_swap_attribute.attr,
	&power_role_swap_attribute.attr,
	&vconn_swap_attribute.attr,
	&vconn_switch_attribute.attr,

	&sop_type_attribute.attr,
	&send_vdm_attribute.attr,
	&get_source_cap_attribute.attr,
	&get_sink_cap_attribute.attr,

	&cc_silicon_id_attribute.attr,
	&cc_fw_version_attribute.attr,
	&cc_running_mode_attribute.attr,
	&cc_boot_reason_attribute.attr,
	&cc_reset_attribute.attr,
	&cc_update_fw_attribute.attr,

	NULL,	/* Need to NULL terminate the list of attributes */
};

static struct kobj_type hpi_port_ktype = {
	.sysfs_ops = &hpi_port_sysfs_ops,
	.release = hpi_port_sysfs_release,
	.default_attrs = hpi_port_sysfs_default_attrs,
};

static int hpi_port_sysfs_init(struct hpi_device *port, struct kobject *parent)
{
	int err;

	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.
	 */
	err = kobject_init_and_add(&port->kobj, &hpi_port_ktype, parent,
				      "port_%d", port->idx);
	if (err) {
		kobject_put(&port->kobj);
		return err;
	}

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	kobject_uevent(&port->kobj, KOBJ_ADD);

	return 0;
}

static void hpi_port_sysfs_deinit(struct hpi_device *port)
{
	kobject_put(&port->kobj);
}

static ssize_t sysfs_show_silicon_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	size = scnprintf(buf, PAGE_SIZE, "0x%04x\n",
				cyccg->ccg_info.silicon_id);

	mutex_unlock(&cyccg->sysfs_mlock);
	return err ? err : size;
}
static DEVICE_ATTR(silicon_id, S_IRUGO, sysfs_show_silicon_id, NULL);

static ssize_t sysfs_show_ccg_generation(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	size = scnprintf(buf, PAGE_SIZE, "CCG%d\n",
				(int)cyccg->ccg_info.ccg_ver);

	mutex_unlock(&cyccg->sysfs_mlock);
	return err ? err : size;
}
static DEVICE_ATTR(ccg_generation, S_IRUGO, sysfs_show_ccg_generation, NULL);

static ssize_t sysfs_show_hpi_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int version_num;
	int size;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	version_num = (int)(cyccg->ccg_info.hpi_ver - HPI_VERSION_1) + 1;
	size = scnprintf(buf, PAGE_SIZE, "HPIv%d\n", version_num);

	mutex_unlock(&cyccg->sysfs_mlock);
	return err ? err : size;
}
static DEVICE_ATTR(hpi_version, S_IRUGO, sysfs_show_hpi_version, NULL);

static ssize_t sysfs_show_running_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct hpi_device_mode device_mode;
	enum ccg_fw_mode_type running_mode;
	int size;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		sysfs_err("failed to read device_mode, %d\n", err);
		goto out;
	}

	running_mode = (enum ccg_fw_mode_type)device_mode.running_mode;
	size = scnprintf(buf, PAGE_SIZE, "%u (%s)\n", running_mode,
			ccg_fw_mode_type_to_string(
				running_mode, cyccg->ccg_info.ccg_ver));
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	return err ? err : size;
}
static DEVICE_ATTR(running_mode, S_IRUGO, sysfs_show_running_mode, NULL);

static ssize_t sysfs_show_fw_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct hpi_ccg_fw_version *ccg_fw_vers;
	int size;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	ccg_fw_vers = &cyccg->ccg_state.ccg_fw_vers;
	err = hpi_read_all_version(cyccg, ccg_fw_vers);
	if (err) {
		sysfs_err("failed to read all version, %d\n", err);
		goto out;
	}

	if (cyccg->ccg_info.hpi_ver >= HPI_VERSION_2) {
		err = hpi_read_fw2_version(cyccg, ccg_fw_vers);
		if (err) {
			sysfs_err("failed to read fw2 version, %d\n", err);
			goto out;
		}
	}

	size = scnprintf(buf, PAGE_SIZE,
			"bootloader base version: %u.%u.%u.%u\n",
			ccg_fw_vers->btldr.base.major,
			ccg_fw_vers->btldr.base.minor,
			ccg_fw_vers->btldr.base.patch_ver,
			ccg_fw_vers->btldr.base.build_number);

	size += scnprintf(buf + size, PAGE_SIZE - size,
			"bootloader app version: %u.%u,%u,%c%c\n",
			ccg_fw_vers->btldr.app.major,
			ccg_fw_vers->btldr.app.minor,
			ccg_fw_vers->btldr.app.external_circuit_ver,
			ccg_fw_vers->btldr.app.name[0],
			ccg_fw_vers->btldr.app.name[1]);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"APP/FW image-1 base version: %u.%u.%u.%u\n",
			ccg_fw_vers->fw1_app.base.major,
			ccg_fw_vers->fw1_app.base.minor,
			ccg_fw_vers->fw1_app.base.patch_ver,
			ccg_fw_vers->fw1_app.base.build_number);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"APP/FW image-1 app version: %u.%u,%u,%c%c\n",
			ccg_fw_vers->fw1_app.app.major,
			ccg_fw_vers->fw1_app.app.minor,
			ccg_fw_vers->fw1_app.app.external_circuit_ver,
			ccg_fw_vers->fw1_app.app.name[0],
			ccg_fw_vers->fw1_app.app.name[1]);
	if (cyccg->ccg_info.hpi_ver >= HPI_VERSION_2) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"FW image-2 base version: %u.%u.%u.%u\n",
				ccg_fw_vers->fw2_app.base.major,
				ccg_fw_vers->fw2_app.base.minor,
				ccg_fw_vers->fw2_app.base.patch_ver,
				ccg_fw_vers->fw2_app.base.build_number);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"FW image-2 app version: %u.%u,%u,%c%c\n",
				ccg_fw_vers->fw2_app.app.major,
				ccg_fw_vers->fw2_app.app.minor,
				ccg_fw_vers->fw2_app.app.external_circuit_ver,
				ccg_fw_vers->fw2_app.app.name[0],
				ccg_fw_vers->fw2_app.app.name[1]);
	}
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	return err ? err : size;
}
static DEVICE_ATTR(fw_version, S_IRUGO, sysfs_show_fw_version, NULL);

static ssize_t sysfs_show_port_list(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	size = cyccg_get_port_list_string(cyccg, buf, PAGE_SIZE);

	mutex_unlock(&cyccg->sysfs_mlock);
	return size;
}
static DEVICE_ATTR(port_list, S_IRUGO, sysfs_show_port_list, NULL);

static ssize_t sysfs_store_update_fw(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(&cyccg->hpi_dev)) {
		sysfs_err("CCG or some PD ports was busying\n");
		err = -EBUSY;
		goto out;
	}

	err = cyccg_update_do_fw_update(cyccg, CCG_NOTEBOOK_MOBILE_MONITOR,
			false, buf, count);
	if (err)
		sysfs_err("failed to do FW update, %d\n", err);

	cyccg_set_to_busying_state(&cyccg->hpi_dev, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static DEVICE_ATTR(update_fw, S_IWUSR | S_IWGRP, NULL, sysfs_store_update_fw);

static ssize_t sysfs_store_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	if (cyccg_busying_check_and_set(&cyccg->hpi_dev)) {
		sysfs_err("CCG or some PD ports was busying\n");
		err = -EBUSY;
		goto out;
	}

	err = cyccg_device_reset_and_init(cyccg, true);
	if (err)
		sysfs_err("failed to do device reset and init, %d\n", err);

	cyccg_set_to_busying_state(&cyccg->hpi_dev, false);
out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ? (err > 0 ? -EFAULT : err) : count;
}
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, sysfs_store_reset);

static ssize_t sysfs_show_reg_addr_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	size = scnprintf(buf, PAGE_SIZE,
			"reg_addr: 0x%04x (%u)\nreg_size: 0x%04x (%zu)\n",
			cyccg->reg_addr, cyccg->reg_addr,
			(unsigned int)cyccg->reg_size, cyccg->reg_size);

	mutex_unlock(&cyccg->sysfs_mlock);
	return size;
}

static ssize_t sysfs_store_reg_addr_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	char **data_array;
	int data_count;
	unsigned long long val;
	int err = 0;

	sysfs_vdbg("sysfs interface called\n");
	data_array = argv_split(GFP_KERNEL, buf, &data_count);
	if (!data_array || !data_count)
		return -EINVAL;

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		err = -EAGAIN;
		goto out;
	}

	if (data_count < 1 || !data_array[0] ||
			kstrtoull(data_array[0], 16, &val)) {
		err = -EINVAL;
		goto err;
	}
	cyccg->reg_addr = (u32)val;

	if (data_count >= 2) {
		if (!data_array[1] || kstrtoull(data_array[1], 16, &val)) {
			err = -EINVAL;
			goto err;
		}

		cyccg->reg_size = (size_t)val;
	}

	sysfs_dbg("reg_addr: 0x%04x (%u); reg_size: 0x%04x (%zu)\n",
		cyccg->reg_addr, cyccg->reg_addr,
		(unsigned int)cyccg->reg_size, cyccg->reg_size);

	if (cyccg->reg_size > HPI_MAX_REG_RW_SIZE) {
		err = -EINVAL;
		sysfs_err("error: reg RW size(%zu) exceed max FW size(%d)\n",
			cyccg->reg_size, HPI_MAX_REG_RW_SIZE);
		goto err;
	}

err:
	mutex_unlock(&cyccg->sysfs_mlock);
out:
	argv_free(data_array);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ?: count;
}
static DEVICE_ATTR(reg_addr_size, S_IRUGO | S_IWUSR | S_IWGRP,
		sysfs_show_reg_addr_size, sysfs_store_reg_addr_size);

static ssize_t sysfs_show_reg_data_hex(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	u8 data[HPI_MAX_REG_RW_SIZE];
	int size;
	int i;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	if (cyccg->reg_size > HPI_MAX_REG_RW_SIZE) {
		sysfs_err("reg_size=%zu exceeded, expected<=%u\n",
			cyccg->reg_size, HPI_MAX_REG_RW_SIZE);
		return -EINVAL;
	}

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		return -EAGAIN;
	}

	err = bus_ops->read(cyccg->dev, HPI_REG_ADDR_SIZE(hpi_ver),
			data, cyccg->reg_size, cyccg->reg_addr);
	if (err) {
		sysfs_err("read reg data on the bus error, %d\n", err);
		goto out;
	}

	sysfs_dump("read data (addr: 0x%04x, size: %zu):\n",
		&data[0], cyccg->reg_size, cyccg->reg_addr, cyccg->reg_size);

	size = scnprintf(buf, PAGE_SIZE, "Reg data (Hex):\n");
	for (i = 0; i < cyccg->reg_size; i++) {
		if (i != 0 && (i % 16) == 0)
			size += scnprintf(buf + size, PAGE_SIZE - size, "\n");

		size += scnprintf(buf + size, PAGE_SIZE - size,
				"%02x ", data[i]);
	}
	if ((i % 16) != 0)
		size += scnprintf(buf + size, PAGE_SIZE - size, "\n");

out:
	mutex_unlock(&cyccg->sysfs_mlock);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ?: size;
}

static ssize_t sysfs_store_reg_data_hex(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	u8 data[HPI_MAX_REG_RW_SIZE];
	char **data_array;
	int data_count = 0;
	unsigned long long val;
	int i;
	int err;

	sysfs_vdbg("sysfs interface called\n");
	data_array = argv_split(GFP_KERNEL, buf, &data_count);
	if (!data_array || !data_count || data_count > HPI_MAX_REG_RW_SIZE)
		return -EINVAL;

	for (i = 0; i < data_count; i++) {
		if (!data_array[i] || kstrtoull(data_array[i], 16, &val) ||
				val > 0xff) {
			err = -EINVAL;
			goto out;
		}

		data[i] = (u8)val;
	}

	sysfs_dump("write data(addr: 0x%04x, size: %d):\n",
		&data[0], data_count, cyccg->reg_addr, data_count);

	if (!mutex_trylock(&cyccg->sysfs_mlock)) {
		sysfs_err("CCG device is busying, please retry later\n");
		err = -EAGAIN;
		goto out;
	}

	err = bus_ops->write(cyccg->dev, HPI_REG_ADDR_SIZE(hpi_ver),
			data, data_count, cyccg->reg_addr);
	if (err)
		sysfs_err("bus write data error, %d\n", err);

	mutex_unlock(&cyccg->sysfs_mlock);
out:
	argv_free(data_array);
	sysfs_vdbg("sysfs interface exit, %d\n", err);
	return err ?: count;
}
static DEVICE_ATTR(reg_data_hex, S_IRUGO | S_IWUSR | S_IWGRP,
		sysfs_show_reg_data_hex, sysfs_store_reg_data_hex);

static struct attribute *cyccg_sysfs_device_entries[] = {
	&dev_attr_ccg_generation.attr,
	&dev_attr_hpi_version.attr,
	&dev_attr_silicon_id.attr,
	&dev_attr_running_mode.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_port_list.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_reset.attr,

	&dev_attr_reg_addr_size.attr,
	&dev_attr_reg_data_hex.attr,

	NULL,
};

static const struct attribute_group cyccg_sysfs_device_group = {
	.attrs = cyccg_sysfs_device_entries,
};

static void cyccg_remove_sysfs(void *data)
{
	struct cyccg *cyccg = data;
	struct kobject *kobj = &cyccg->dev->kobj;
	struct hpi_device *port;
	int i;

	sysfs_vdbg("<<<< enter\n");
	for (i = 0; i < cyccg->ccg_info.num_port; i++) {
		port = cyccg->ports[i];
		if (!port || port->dev_type < HPI_DEV_TYPE_PORT)
			continue;

		hpi_port_sysfs_deinit(port);
	}

	sysfs_remove_group(kobj, &cyccg_sysfs_device_group);
}

int cyccg_sysfs_init(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct kobject *kobj = &dev->kobj;
	struct hpi_device *port;
	int i, j;
	int err;

	sysfs_vdbg("<<<< enter\n");
	err = sysfs_create_group(kobj, &cyccg_sysfs_device_group);
	if (err) {
		sysfs_err("failed to create sysfs group, %d\n", err);
		return err;
	}

	for (i = 0; i < cyccg->ccg_info.num_port; i++) {
		port = cyccg->ports[i];
		if (!port || port->dev_type < HPI_DEV_TYPE_PORT)
			continue;

		port->sysfs_sop_type = VDM_SOP_TYPE_SOP;  /* default SOP type */
		err = hpi_port_sysfs_init(port, kobj);
		if (err) {
			/* Deinit the sysfs for the initialized port. */
			for (j = 0; j < i; j++)
				hpi_port_sysfs_deinit(cyccg->ports[j]);
			sysfs_remove_group(kobj, &cyccg_sysfs_device_group);

			sysfs_err("init default port_%d attributes error, %d\n",
				port->idx, err);
			return err;
		}
	}

	err = devm_add_action(dev, cyccg_remove_sysfs, cyccg);
	if (err) {
		cyccg_remove_sysfs(cyccg);
		sysfs_err("devm_add_action failed, %d\n", err);
		return err;
	}

	sysfs_vdbg(">>>> exit, success\n");
	return 0;
}