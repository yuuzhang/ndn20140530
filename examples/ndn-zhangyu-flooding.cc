/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2012 University of California, Los Angeles
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
 * Author: Alexander Afanasyev <alexander.afanasyev@ucla.edu>
 */
// ndn-congestion-topo-plugin.cc
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"

//ZhangYu 2013-8-16  for ndn::CsTracer
#include <ns3/ndnSIM/utils/tracers/ndn-cs-tracer.h>
// for ndn::L3AggregateTracer
#include <ns3/ndnSIM/utils/tracers/ndn-l3-aggregate-tracer.h>
// for ndn::AppDelayTracer
#include <ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.h>
// for ndn::L2RateTracer
#include <ns3/ndnSIM/utils/tracers/l2-rate-tracer.h>

// for LinkStatusControl::FailLinks and LinkStatusControl::UpLinks
#include "ns3/ndn-link-control-helper.h"


// for ndn::L3RateTracer
#include <ns3/ndnSIM/utils/tracers/ndn-l3-rate-tracer.h>
//ZhangYu 2014-2-7 for DynamicRouting，否则不认识Name，试了很多.h才知道要包含ndn-interest.h
#include "ns3/names.h"
#include "ns3/ndn-name.h"
#include "string.h"
#include "ns3/ndn-interest.h"
#include "ns3/ptr.h"
#include <boost/ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

using namespace std;

//---ZhangYu

using namespace ns3;

/**
 * ZhangYu 2014-3-10，使用BRITE产生的随机网络拓扑，业务量均匀分布在随机的节点对之间
 * 我们这里可以偷懒，因为节点的位置和链路是随机的，所以可以让节点对编号是连续的，效果上应该和均匀分布的随机是一样的。
 * 而且这样拓扑和业务分布可以是固定的，分析数据结果时，可比性强。
 */

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  AnnotatedTopologyReader topologyReader ("", 20);

  topologyReader.SetFileName ("src/ndnSIM/examples/topologies/26node-result-1.txt");
  //topologyReader.SetFileName ("src/ndnSIM/examples/topologies/topo-for-CompareMultiPath.txt");

  bool manualAssign=false;
  int InterestPerSec=200;
  int simulationSpan=50;

  topologyReader.Read ();
  int nodesNumber=topologyReader.GetNodes().size();

  // Install NDN stack on all nodes
  ndn::StackHelper ndnHelper;
  ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
  //ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
  //ndnHelper.SetForwardingStrategy("ns3::ndn::fw::SmartFloodng");

  ndnHelper.SetContentStore ("ns3::ndn::cs::Lru","MaxSize", "1");
  ndnHelper.InstallAll ();
  topologyReader.ApplyOspfMetric();  //使得链路metric生效

  // Installing global routing interface on all nodes
  ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
  ndnGlobalRoutingHelper.InstallAll ();

  //设置和安装业务
  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
  consumerHelper.SetAttribute ("Frequency", StringValue (boost::lexical_cast<std::string>(InterestPerSec))); // 100 interests a second

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetAttribute ("PayloadSize", StringValue("1024"));


  std::vector<int> consumerNodes,producerNodes;

  //生成consumer和producer的节点号动态数组
  if(manualAssign)
  {
	  int tmpConsumer[]={0,2};
	  int tmpProducer[]={4,3};
	  consumerNodes.assign(tmpConsumer,tmpConsumer+sizeof(tmpConsumer)/sizeof(int));
	  producerNodes.assign(tmpProducer,tmpProducer+sizeof(tmpConsumer)/sizeof(int));
  }
  else
  {
	  for(int i=0;i<nodesNumber/2;i++)
	  {
		  consumerNodes.push_back(i);
		  producerNodes.push_back(i+nodesNumber/2);
	  }

	  for(uint32_t i=0;i<consumerNodes.size();i++)
	  {
		  std::cout << consumerNodes[i];
	  }
  }
  //根据得到的consumer和producer节点号向量生成节点
  for(uint32_t i=0;i<consumerNodes.size();i++)
  {
	  // Getting containers for the consumer/producer
	  Ptr<Node> consumer1 = Names::Find<Node> ("Node"+boost::lexical_cast<std::string> (consumerNodes[i]));
	  // that will express interests in /dst1 namespace
	  consumerHelper.SetPrefix ("/Node"+boost::lexical_cast<std::string>(consumerNodes[i]));
	  consumerHelper.Install (consumer1);
	  std::cout <<"ZhangYu 2014-3-7 consumer1->GetId(): " <<consumer1->GetId() << std::endl;
  }

  for(uint32_t i=0;i<producerNodes.size();i++)
  {
	  //认为producer节点的Prefix和对应位置的consumer节点一致
	  producerHelper.SetPrefix ("/Node"+boost::lexical_cast<std::string>(consumerNodes[i]));
	  Ptr<Node> producer1 = Names::Find<Node> ("Node"+boost::lexical_cast<std::string> (producerNodes[i]));
	  // install producer that will satisfy Interests in /dst1 namespace
	  ndnGlobalRoutingHelper.AddOrigins ("/Node"+boost::lexical_cast<std::string>(consumerNodes[i]), producer1);
	  producerHelper.Install(producer1);
	  std::cout <<"ZhangYu 2014-3-7 producer1->GetId(): " <<producer1->GetId() << std::endl;
  }


  // Calculate and install FIBs
  //ndn::GlobalRoutingHelper::CalculateRoutes ();
  ndn::GlobalRoutingHelper::CalculateAllPossibleRoutes();
  //ndn::GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes();

  // The failure of the link connecting consumer and router will start from seconds 10.0 to 15.0
  //Simulator::Schedule (Seconds (10.0), ndn::LinkControlHelper::FailLink, Names::Find<Node> ("Node0"),Names::Find<Node> ("Node4"));
  //Simulator::Schedule (Seconds (15.0), ndn::LinkControlHelper::UpLink,   Names::Find<Node> ("Node0"),Names::Find<Node> ("Node4"));

  Simulator::Stop (Seconds (simulationSpan));

  //ZhangYu Add the trace
  ndn::CsTracer::InstallAll ("cs-trace-flooding.txt", Seconds (1));
  ndn::L3AggregateTracer::InstallAll ("aggregate-trace-flooding.txt", Seconds (1));
  ndn::L3RateTracer::InstallAll ("rate-trace-flooding.txt", Seconds (1));
  ndn::AppDelayTracer::InstallAll ("app-delays-trace-flooding.txt");
  L2RateTracer::InstallAll ("drop-trace-flooding.txt", Seconds (1));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
