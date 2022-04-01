/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013,2014 TELEMATICS LAB, DEI - Politecnico di Bari
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Giuseppe Piro <peppe@giuseppepiro.com>, <g.piro@poliba.it>
 */


#include "traditional-nano-routing-entity.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/packet.h"
#include "simple-nano-device.h"
#include "nano-mac-queue.h"
#include "nano-l3-header.h"
#include "nano-mac-entity.h"
#include "ns3/log.h"
#include "ns3/queue.h"
#include "ns3/simulator.h"
#include "ns3/enum.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/pointer.h"
#include "ns3/channel.h"
#include "simple-nano-device.h"
#include "nano-spectrum-phy.h"
#include "nano-mac-entity.h"
#include "nano-mac-header.h"
#include "nano-seq-ts-header.h"
#include "ns3/simulator.h"
#include "nano-routing-entity.h"
#include "message-process-unit.h"


NS_LOG_COMPONENT_DEFINE ("TraditionalNanoRoutingEntity");

namespace ns3 {


NS_OBJECT_ENSURE_REGISTERED (TraditionalNanoRoutingEntity);

TypeId TraditionalNanoRoutingEntity::GetTypeId(void) {
	static TypeId tid = TypeId("ns3::TraditionalNanoRoutingEntity").SetParent<Object>();
	return tid;
}


TraditionalNanoRoutingEntity::TraditionalNanoRoutingEntity ()
{
	SetDevice(0);
	m_receivedPacketListDim = 100;
	for (int i = 0; i < m_receivedPacketListDim; i++)
	{
		m_receivedPacketList.push_back (9999999);
	}
}


TraditionalNanoRoutingEntity::~TraditionalNanoRoutingEntity ()
{
	SetDevice(0);
}

void  TraditionalNanoRoutingEntity::DoDispose (void)
{
	SetDevice (0);
}

void TraditionalNanoRoutingEntity::SendPacket (Ptr<Packet> p)
{
	if(GetDevice()->GetEnergyCapacity() >= GetDevice()->GetMinSatisfidSendEnergy()) {			//节点能量满足最小发送能量
		GetDevice()->ConsumeEnergySend(GetDevice()->GetPacketSize());			//节点消耗发送数据包的能量，获取周围的邻居节点，此处过程简写，能量是否足够判断已在创建数据包时判断

		NanoSeqTsHeader seqTs;				//seqTs的size为12字节
		seqTs.SetSeq (p->GetUid ());
		p->AddHeader (seqTs);				//p->GetSize()为初始数据包大小+12

		NanoL3Header l3Header;
		l3Header.SetSource(GetDevice()->GetNode()->GetId());
		l3Header.SetDestination(0);						//目的地固定为0，即网关节点
		l3Header.SetTtl(15);
		l3Header.SetPacketId(p->GetUid());
		l3Header.Setindex(GetDevice ()->index);
		p->AddHeader(l3Header);

		UpdateReceivedPacketId(p->GetUid());				//自己发送的自己不需要接收，也放入接收过的数据包队列
		std::vector<NanoDetail>::iterator it;
		GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//纳米节点发送出去后无论是否能被路由节点接收都丢弃该数据包
		GetDevice()->GetMac()->Send(p);				//发送一遍即可，周围邻居节点都会收到
	} else {	 		//没有能量,间隔reSendTimeInterval之后再次调用该节点的mac协议发送
		std::cout << GetDevice()->GetNode()->GetId() << "   " << "SendPacket no energy to send" << std::endl;
		Simulator::Schedule (Seconds (GetDevice ()->reSendTimeInterval), &TraditionalNanoRoutingEntity::SendPacket, this, p);			//0.1s
	}
}

void TraditionalNanoRoutingEntity::ReceivePacket (Ptr<Packet> p, Ptr<SpectrumPhy> sourcePhy)
{
	NanoMacHeader macHeader;					//注意：需要按照堆栈的顺序来取，不能弄错，mac后放得先取
	p->RemoveHeader(macHeader);
	NanoL3Header l3Header;
	p->RemoveHeader(l3Header);

	bool alreadyReceived = CheckAmongReceivedPacket(p->GetUid());
	if (GetDevice()->m_type == SimpleNanoDevice::NanoInterface) {				//如果是接收节点是网关节点
		if (!alreadyReceived) {			//如果没有接收过该数据包，则网关节点处理该数据包
			GetDevice()->GetObject<SimpleNanoDevice>()->ConsumeEnergyReceive(GetDevice()->GetPacketSize());		//网关节点接收数据包消耗能量
			p->AddHeader(l3Header);
			UpdateReceivedPacketId(p->GetUid());				//将数据包id记录队列，防止重复接收

			double distance = GetDevice ()->GetPhy()->GetMobility()->GetDistanceFrom(sourcePhy->GetMobility());			//由于有坐标系可直接计算纳米节点与路由节点的距离，不用通过传输时间计算
			uint32_t distanceTxTime = distance / 300000000 * 1e15;

			std::cout << Simulator::Now().GetSeconds() << " " << "ProcessMessage" << "   " << "packetId:" << p->GetUid() << "   " << "sourcePhy:" << sourcePhy->GetDevice()->GetNode()->GetId() << " ----------------------> " << GetDevice()->GetNode()->GetId() << std::endl;
			GetDevice()->GetMessageProcessUnit()->ProcessMessage(p, distanceTxTime);
		} else {						//遇到重复的数据包需要清除该节点数据包，方便下次产生数据包，否则该节点数据包一直不会清除
			std::cout << Simulator::Now().GetSeconds() << " " << "###clear replicate packet###" << " " << "NodeId:" << sourcePhy->GetDevice()->GetNode()->GetId() << "   " << "packetId:" << p->GetUid() << std::endl;
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//自身数据包被别的节点接收，标志位置为true，节点此时没有数据包，可以进行转发别的节点数据包
		}
	} else if(GetDevice()->m_type == SimpleNanoDevice::NanoNode){					//如果是接收节点是普通纳米节点
		if(GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag) {	//所有携带数据包的纳米节点将打开接收通道，所以会接收到其它节点广播的数据包，没有携带数据包的节点静默
			GetDevice()->GetObject<SimpleNanoDevice>()->ConsumeEnergyReceive(GetDevice()->GetPacketSize());		//纳米节点接收数据包消耗能量
		}
	}
}

void TraditionalNanoRoutingEntity::UpdateReceivedPacketId (uint32_t id)			//弹出一个最早的数据，push一个新数据
{
	m_receivedPacketList.pop_front ();				//pop一个又push一个，保持m_receivedPacketList的大小为20
	m_receivedPacketList.push_back (id);
}

bool TraditionalNanoRoutingEntity::CheckAmongReceivedPacket (uint32_t id)
{
	for (std::list<uint32_t>::iterator it = m_receivedPacketList.begin(); it != m_receivedPacketList.end (); it++)
	{
		NS_LOG_FUNCTION (this << *it << id);
		if (*it == id) return true;
	}
	return false;
}

void TraditionalNanoRoutingEntity::SetReceivedPacketListDim (int m) {
	NS_LOG_FUNCTION (this);
	m_receivedPacketListDim = m;
}

} // namespace ns3
