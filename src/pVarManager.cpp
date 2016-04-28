/*
 * pVarManager.cpp
 *
 * Author: Petel__
 * Version: 3.2.4
 *
 * Copyright (c) 2014-2016 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstdlib>
#include <functional>
#include <utility>

#include <libsc/system.h>
#include <libsc/k60/ftdi_ft232r.h>
#include <libbase/k60/sys_tick.h>
#include <libsc/k60/jy_mcu_bt_106.h>

#include "pVarManager.h"

//#ifdef LIBSC_USE_UART

using namespace libbase::k60;
using namespace libsc;
using namespace libsc::k60;
using namespace std;

pVarManager *m_pd_instance;

pVarManager::TypeId *pVarManager::TypeId::m_instance = nullptr;

inline uint8_t inRangeWithAssert(uint8_t n, uint8_t v, uint8_t x)
{
	if (n <= v && x > v)
		return v;
	else
	{
		assert(false);
		return -1;
	}
}

JyMcuBt106::Config pVarManager::get106UartConfig(const uint8_t id)
{
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.rx_irq_threshold = rx_threshold;
	config.is_rx_irq_threshold_percentage = false;
	config.tx_buf_size = 50;
	config.rx_isr = &listener;
	return config;
}

FtdiFt232r::Config pVarManager::get232UartConfig(const uint8_t id)
{
	FtdiFt232r::Config config;
	config.id = id;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.rx_irq_threshold = rx_threshold;
	config.is_rx_irq_threshold_percentage = false;
	config.tx_buf_size = 50;
	return config;
}

pVarManager::pVarManager(void)
:
	isStarted(false),
	rx_threshold(7),
	m_uart(get106UartConfig(0))
{
	m_pd_instance = this;
	System::Init();
	TypeId::Init();
}

pVarManager::~pVarManager()
{
	sharedObjMng.clear();
	watchedObjMng.clear();
}

// data: [ cmd, isInternalCmd, targetSharedVarIndex / enabledWatchedVarIndice (binary representation), value0,...,value3 ];
bool pVarManager::listener(const Byte *data, const size_t size)
{
	vector<Byte> bytes(data, data + size);
	m_pd_instance->rx_buffer.insert(m_pd_instance->rx_buffer.end(), bytes.begin(), bytes.end());

	if (m_pd_instance->rx_buffer.size() < m_pd_instance->rx_threshold)
		return true;

	if (m_pd_instance->rx_buffer.size() >= m_pd_instance->rx_threshold)
	{
		if (m_pd_instance->rx_buffer[1])
		{
			switch (m_pd_instance->rx_buffer[0])
			{
			case 's':
				m_pd_instance->isStarted = true;
				break;

			case 'e':
				m_pd_instance->isStarted = false;
				break;

			case 'w':
				m_pd_instance->sendWatchedVarInfo();
				break;

			case 'h':
				m_pd_instance->sendSharedVarInfo();
				break;

			case 'c':
				m_pd_instance->changeSharedVars(m_pd_instance->rx_buffer);
				break;
			}
		}
		else
			m_pd_instance->m_origin_listener(m_pd_instance->rx_buffer);
	}

	m_pd_instance->rx_buffer.erase(m_pd_instance->rx_buffer.begin(), m_pd_instance->rx_buffer.begin() + 7);
	return true;
}

void pVarManager::sendWatchData(void)
{
	if (isStarted)
		for (Byte i = 0; i < watchedObjMng.size(); i++)
			m_uart.SendBuffer((Byte *)((ObjMng)watchedObjMng.at(i)).obj, ((ObjMng)watchedObjMng.at(i)).len);
}

void pVarManager::sendWatchedVarInfo(void)
{
	m_uart.SendBuffer((Byte *)",", 1);
	Byte n = watchedObjMng.size();
	m_uart.SendBuffer((Byte *)&n, 1);
	for (Byte i = 0; i < watchedObjMng.size(); i++)
	{
		ObjMng temp = watchedObjMng.at(i);
		m_uart.SendBuffer((Byte *)temp.typeName.data(), temp.typeName.size() + 1);
		m_uart.SendBuffer((Byte *)temp.varName.data(), temp.varName.size() + 1);
		m_uart.SendBuffer((Byte *)",", 1);
	}
	m_uart.SendBuffer((Byte *)"end", 3);
}

void pVarManager::sendSharedVarInfo(void)
{
	m_uart.SendBuffer((Byte *)".", 1);
	Byte n = sharedObjMng.size();
	m_uart.SendBuffer((Byte *)&n, 1);
	for (Byte i = 0; i < sharedObjMng.size(); i++)
	{
		ObjMng temp = sharedObjMng.at(i);
		m_uart.SendBuffer((Byte *)temp.typeName.data(), temp.typeName.size() + 1);
		m_uart.SendBuffer((Byte *)temp.varName.data(), temp.varName.size() + 1);
		m_uart.SendBuffer((Byte *)temp.obj, temp.len);
		m_uart.SendBuffer((Byte *)".", 1);
	}
	m_uart.SendBuffer((Byte *)"end", 3);
}

void pVarManager::changeSharedVars(const std::vector<Byte> &msg)
{
	int objIndex = inRangeWithAssert(0, msg[2], sharedObjMng.size());
	for (int i = 0; i < sharedObjMng[objIndex].len; i++)
		((Byte *)(sharedObjMng[objIndex].obj))[i] = msg[3 + i];

	if (m_onChanged_listener)
		m_onChanged_listener();
}

void pVarManager::SetOnReceiveListener(const OnReceiveListener &oriListener)
{
	if (!isStarted)
		if (!m_origin_listener)
			m_origin_listener = oriListener;
}

void pVarManager::SetOnChangedListener(const OnChangedListener &changedlistener)
{
	if (!isStarted)
		if (!m_onChanged_listener)
			m_onChanged_listener = changedlistener;
}

void pVarManager::RemoveOnReceiveListener(void)
{
	m_origin_listener = nullptr;
}

void pVarManager::RemoveOnChangedListener(void)
{
	m_origin_listener = nullptr;
}

void pVarManager::RemoveAllListeners(void)
{
	m_origin_listener = nullptr;
	m_origin_listener = nullptr;
}

//#endif /* LIBSC_USE_UART */
