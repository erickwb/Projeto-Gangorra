#ifndef BLUETOOTH_HPP
#define BLUETOOTH_HPP

#include "system.hpp"

using BluetoothHandle = void*;

extern void vInitBluetooth(void);

extern void vAwaitBluetoothConnection(BluetoothHandle *handle);

extern void vReceiveBluetoothData(BluetoothHandle *handle);

#endif