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


#include "flooding-nano-routing-entity.h"
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


NS_LOG_COMPONENT_DEFINE ("FloodingNanoRoutingEntity");

namespace ns3 {


NS_OBJECT_ENSURE_REGISTERED (FloodingNanoRoutingEntity);

TypeId FloodingNanoRoutingEntity::GetTypeId(void) {
	static TypeId tid = TypeId("ns3::FloodingNanoRoutingEntity").SetParent<Object>();
	return tid;
}


FloodingNanoRoutingEntity::FloodingNanoRoutingEntity ()
{
	SetDevice(0);
	m_receivedPacketListDim = 100;
	for (int i = 0; i < m_receivedPacketListDim; i++)
	{
		m_receivedPacketList.push_back (9999999);
	}
}


FloodingNanoRoutingEntity::~FloodingNanoRoutingEntity ()
{
	SetDevice(0);
}

void  FloodingNanoRoutingEntity::DoDispose (void)
{
	SetDevice (0);
}

void FloodingNanoRoutingEntity::SendPacket (Ptr<Packet> p)
{
	if(GetDevice()->GetEnergyCapacity() >= GetDevice()->GetMinSatisfidSendEnergy()) {			//????????????????????????????????????
		GetDevice()->ConsumeEnergySend(GetDevice()->GetTestSize());			//??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
		GetDevice ()->GetMac ()->CheckForNeighborss(p);
		std::vector<NanoDetail> neighbors = GetDevice ()->GetMac ()->m_neighborss;
		std::cout << Simulator::Now().GetSeconds() << " " << "GetNode()->GetId():" << GetDevice ()->GetNode()->GetId() << "   " << "neighbors.size ():" << neighbors.size () << "   " << "index:" << GetDevice()->index  << std::endl;

		if (neighbors.size() != 0) {
			GetDevice()->ConsumeEnergyReceive(GetDevice()->GetTestSize() * neighbors.size());				//???????????????????????????????????????????????????????????????????????????

			NanoL3Header l3Header;
			l3Header.SetSource(GetDevice()->GetNode()->GetId());
			l3Header.SetDestination(0);						//??????????????????0??????????????????
			l3Header.SetTtl(15);
			l3Header.SetPacketId(p->GetUid());
			l3Header.Setindex(GetDevice ()->index);
			p->AddHeader(l3Header);

			UpdateReceivedPacketId(p->GetUid());				//???????????????????????????????????????????????????????????????????????????
			GetDevice()->ConsumeEnergySend(GetDevice()->GetPacketSize());				//???????????????????????????????????????????????????????????????????????????????????????????????????
			std::vector<NanoDetail>::iterator it;
			GetDevice()->GetMac()->Send(p);				//???????????????????????????????????????????????????
		} else {	 		//??????????????????	,??????reSendTimeInterval??????????????????????????????mac????????????
			Simulator::Schedule (Seconds (GetDevice ()->reSendTimeInterval), &FloodingNanoRoutingEntity::SendPacket, this, p);			//0.1s
		}
	} else {	 		//????????????,??????reSendTimeInterval??????????????????????????????mac????????????
		std::cout << GetDevice()->GetNode()->GetId() << "   " << "SendPacket no energy to send" << std::endl;
		Simulator::Schedule (Seconds (GetDevice ()->reSendTimeInterval), &FloodingNanoRoutingEntity::SendPacket, this, p);			//0.1s
	}
}

void FloodingNanoRoutingEntity::ReceivePacket (Ptr<Packet> p, Ptr<SpectrumPhy> sourcePhy)
{
	NanoMacHeader macHeader;					//????????????????????????????????????????????????????????????mac???????????????
	p->RemoveHeader(macHeader);
	NanoL3Header l3Header;
	p->RemoveHeader(l3Header);

	bool alreadyReceived = CheckAmongReceivedPacket(p->GetUid());
	if (GetDevice()->m_type == SimpleNanoDevice::NanoInterface) {				//????????????????????????????????????
		if (!alreadyReceived) {			//?????????????????????????????????????????????????????????????????????
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->ConsumeEnergyReceive(GetDevice()->GetAckSize());		//????????????????????????????????????ACK???????????????
			p->AddHeader(l3Header);
			UpdateReceivedPacketId(p->GetUid());				//????????????id?????????????????????????????????
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//??????????????????????????????????????????????????????true????????????????????????????????????????????????????????????????????????

			//????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->EvendJudge();
			std::cout << Simulator::Now().GetSeconds() << " " << "ProcessMessage" << "   " << "packetId:" << p->GetUid() << "   " << "sourcePhy:" << sourcePhy->GetDevice()->GetNode()->GetId() << " ----------------------> " << GetDevice()->GetNode()->GetId() << std::endl;
			GetDevice()->GetMessageProcessUnit()->ProcessMessage(p);
		} else {						//?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
			std::cout << Simulator::Now().GetSeconds() << " " << "###clear replicate packet###" << " " << "NodeId:" << sourcePhy->GetDevice()->GetNode()->GetId() << "   " << "packetId:" << p->GetUid() << std::endl;
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//??????????????????????????????????????????????????????true????????????????????????????????????????????????????????????????????????

			//????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->EvendJudge();
		}
	} else if(GetDevice()->m_type == SimpleNanoDevice::NanoNode){					//??????????????????????????????????????????
		GetDevice()->ConsumeEnergyReceive(GetDevice()->GetPacketSize());				//????????????????????????????????????????????????????????????
		if (!alreadyReceived) {
			std::vector<NanoDetail> nowNeighbors = sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->GetMac()->m_neighborss;
			std::vector<NanoDetail>::iterator it;
			for (it = nowNeighbors.begin (); it != nowNeighbors.end (); it++) {
				//?????????????????????????????????????????????id????????????????????????????????????
				if((*it).detail_id == GetDevice()->GetNode()->GetId() && GetDevice()->packetExistFlag == false && GetDevice()->m_energy >= GetDevice()->GetMinSatisfidForwardEnergy()) {
					GetDevice()->ConsumeEnergySend(GetDevice()->GetAckSize());				//????????????????????????????????????ACK???????????????
					sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->ConsumeEnergyReceive(GetDevice()->GetAckSize());		//????????????????????????????????????ACK???????????????
					p->AddHeader(l3Header);
					UpdateReceivedPacketId(p->GetUid());				//????????????id?????????????????????????????????
					sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//??????????????????????????????????????????????????????true????????????????????????????????????????????????????????????????????????

					//????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
					sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->EvendJudge();
					GetDevice ()->packetExistFlag = true;			//????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????false
					std::cout << Simulator::Now().GetSeconds() << " " << "forward" << "   " << "packetId:" << p->GetUid() << "   " << "ttl:" << l3Header.GetTtl() << "   " << "sourcePhy:" << sourcePhy->GetDevice()->GetNode()->GetId() << " ------------------------> " << GetDevice()->GetNode()->GetId() << std::endl;
					ForwardPacket(p);
				}
			}
		} else {
			//std::cout << Simulator::Now().GetSeconds() << " " << "alreadyReceived" << "   " << "sourceId:" << sourcePhy->GetDevice()->GetNode()->GetId() << "   " << "NodeId:" << GetDevice()->GetNode()->GetId() << "   " << "packetId:" << p->GetUid() << std::endl;
		}
	}
}

void FloodingNanoRoutingEntity::ForwardPacket (Ptr<Packet> p)
{
	if(GetDevice()->GetEnergyCapacity() >= GetDevice()->GetMinSatisfidSendEnergy()) {			//????????????????????????????????????
		GetDevice()->ConsumeEnergySend(GetDevice()->GetTestSize());			//??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
		GetDevice ()->GetMac ()->CheckForNeighborss(p);
		std::vector<NanoDetail> neighbors = GetDevice ()->GetMac ()->m_neighborss;
		std::cout << Simulator::Now().GetSeconds() << " " << "GetNode()->GetId():" << GetDevice ()->GetNode()->GetId() << "   " << "neighbors.size ():" << neighbors.size () << "   " << "index:" << GetDevice()->index << std::endl;

		if (neighbors.size() != 0) {
			GetDevice()->ConsumeEnergyReceive(GetDevice()->GetTestSize() * neighbors.size());				//???????????????????????????????????????????????????????????????????????????

			NanoL3Header l3Header;
			p->RemoveHeader(l3Header);
			uint32_t ttl = l3Header.GetTtl();
			if (ttl > 1) {
				l3Header.SetTtl(ttl - 1);
				p->AddHeader(l3Header);

				std::vector<NanoDetail>::iterator it;
				GetDevice()->ConsumeEnergySend(GetDevice()->GetPacketSize());				//??????????????????????????????????????????????????????????????????????????????????????????????????????
				GetDevice()->GetMac()->Send(p);
			} else {
				std::cout << Simulator::Now().GetSeconds() << " " << "TTL expire" << "   " << "ttl:" << l3Header.GetTtl() << "   " << "GetId:" << GetDevice()->GetNode()->GetId() << "   " << "packetId:" << p->GetUid() << std::endl;
				GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;				//?????????TTL?????????????????????

				//????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
				GetDevice()->GetObject<SimpleNanoDevice>()->EvendJudge();
			}
		} else {	 		//??????????????????	,??????reSendTimeInterval??????????????????????????????mac????????????
			Simulator::Schedule (Seconds (GetDevice ()->reSendTimeInterval), &FloodingNanoRoutingEntity::ForwardPacket, this, p);			//0.1s
		}
	} else {	 		//????????????,??????reSendTimeInterval??????????????????????????????mac????????????
		std::cout << GetDevice()->GetNode()->GetId() << "   " << "ForwardPacket no energy to send" << std::endl;
		Simulator::Schedule (Seconds (GetDevice ()->reSendTimeInterval), &FloodingNanoRoutingEntity::ForwardPacket, this, p);			//0.1s
	}
}

void FloodingNanoRoutingEntity::UpdateReceivedPacketId (uint32_t id)			//??????????????????????????????push???????????????
{
	m_receivedPacketList.pop_front ();				//pop?????????push???????????????m_receivedPacketList????????????20
	m_receivedPacketList.push_back (id);
}

bool FloodingNanoRoutingEntity::CheckAmongReceivedPacket (uint32_t id)
{
	for (std::list<uint32_t>::iterator it = m_receivedPacketList.begin(); it != m_receivedPacketList.end (); it++)
	{
		NS_LOG_FUNCTION (this << *it << id);
		if (*it == id) return true;
	}
	return false;
}

void FloodingNanoRoutingEntity::SetReceivedPacketListDim (int m) {
	NS_LOG_FUNCTION (this);
	m_receivedPacketListDim = m;
}

} // namespace ns3
