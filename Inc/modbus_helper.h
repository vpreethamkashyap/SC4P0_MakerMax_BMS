#ifndef MODBUS_HELPER_H
#define MODBUS_HELPER_H

void enable_modbusrtu(void);
void updateModbusInputRegisters();
void modbus_task_init(void);
void modbus_task(void);

#endif
