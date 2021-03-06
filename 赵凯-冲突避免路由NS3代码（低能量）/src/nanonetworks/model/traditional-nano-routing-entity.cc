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
	if(GetDevice()->GetEnergyCapacity() >= GetDevice()->GetMinSatisfidSendEnergy()) {			//????????????????????????????????????
		GetDevice()->ConsumeEnergySend(GetDevice()->GetPacketSize());			//????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

		NanoSeqTsHeader seqTs;				//seqTs???size???12??????
		seqTs.SetSeq (p->GetUid ());
		p->AddHeader (seqTs);				//p->GetSize()????????????????????????+12

		NanoL3Header l3Header;
		l3Header.SetSource(GetDevice()->GetNode()->GetId());
		l3Header.SetDestination(0);						//??????????????????0??????????????????
		l3Header.SetTtl(15);
		l3Header.SetPacketId(p->GetUid());
		l3Header.Setindex(GetDevice ()->index);
		p->AddHeader(l3Header);

		UpdateReceivedPacketId(p->GetUid());				//???????????????????????????????????????????????????????????????????????????
		std::vector<NanoDetail>::iterator it;
		GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//????????????????????????????????????????????????????????????????????????????????????
		GetDevice()->GetMac()->Send(p);				//???????????????????????????????????????????????????
	} else {	 		//????????????,??????reSendTimeInterval??????????????????????????????mac????????????
		std::cout << GetDevice()->GetNode()->GetId() << "   " << "SendPacket no energy to send" << std::endl;
		Simulator::Schedule (Seconds (GetDevice ()->reSendTimeInterval), &TraditionalNanoRoutingEntity::SendPacket, this, p);			//0.1s
	}
}

void TraditionalNanoRoutingEntity::ReceivePacket (Ptr<Packet> p, Ptr<SpectrumPhy> sourcePhy)
{
	NanoMacHeader macHeader;					//????????????????????????????????????????????????????????????mac???????????????
	p->RemoveHeader(macHeader);
	NanoL3Header l3Header;
	p->RemoveHeader(l3Header);

	bool alreadyReceived = CheckAmongReceivedPacket(p->GetUid());
	if (GetDevice()->m_type == SimpleNanoDevice::NanoInterface) {				//????????????????????????????????????
		if (!alreadyReceived) {			//?????????????????????????????????????????????????????????????????????
			GetDevice()->GetObject<SimpleNanoDevice>()->ConsumeEnergyReceive(GetDevice()->GetPacketSize());		//???????????????????????????????????????
			p->AddHeader(l3Header);
			UpdateReceivedPacketId(p->GetUid());				//????????????id?????????????????????????????????

			double distance = GetDevice ()->GetPhy()->GetMobility()->GetDistanceFrom(sourcePhy->GetMobility());			//??????????????????????????????????????????????????????????????????????????????????????????????????????
			uint32_t distanceTxTime = distance / 300000000 * 1e15;

			std::cout << Simulator::Now().GetSeconds() << " " << "ProcessMessage" << "   " << "packetId:" << p->GetUid() << "   " << "sourcePhy:" << sourcePhy->GetDevice()->GetNode()->GetId() << " ----------------------> " << GetDevice()->GetNode()->GetId() << std::endl;
			GetDevice()->GetMessageProcessUnit()->ProcessMessage(p, distanceTxTime);
		} else {						//?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
			std::cout << Simulator::Now().GetSeconds() << " " << "###clear replicate packet###" << " " << "NodeId:" << sourcePhy->GetDevice()->GetNode()->GetId() << "   " << "packetId:" << p->GetUid() << std::endl;
			sourcePhy->GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag = false;			//??????????????????????????????????????????????????????true????????????????????????????????????????????????????????????????????????
		}
	} else if(GetDevice()->m_type == SimpleNanoDevice::NanoNode){					//??????????????????????????????????????????
		if(GetDevice()->GetObject<SimpleNanoDevice>()->packetExistFlag) {	//???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
			GetDevice()->GetObject<SimpleNanoDevice>()->ConsumeEnergyReceive(GetDevice()->GetPacketSize());		//???????????????????????????????????????
		}
	}
}

void TraditionalNanoRoutingEntity::UpdateReceivedPacketId (uint32_t id)			//??????????????????????????????push???????????????
{
	m_receivedPacketList.pop_front ();				//pop?????????push???????????????m_receivedPacketList????????????20
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
