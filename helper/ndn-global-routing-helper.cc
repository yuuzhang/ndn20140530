/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil -*- */
/*
 * Copyright (c) 2011 UCLA
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
 * Author:  Alexander Afanasyev <alexander.afanasyev@ucla.edu>
 */
//-----ZhangYu 2013-12-29 为了修改MultiPath添加的头文件
#include "ns3/ndn-app-helper.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ndn-app.h"
using namespace ns3;
//---------

#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunneeded-internal-declaration"
#endif

#include "ndn-global-routing-helper.h"

#include "ns3/ndn-l3-protocol.h"
#include "../model/ndn-net-device-face.h"
#include "../model/ndn-global-router.h"
#include "ns3/ndn-name.h"
#include "ns3/ndn-fib.h"

#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/net-device.h"
#include "ns3/channel.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/names.h"
#include "ns3/node-list.h"
#include "ns3/channel-list.h"
#include "ns3/object-factory.h"

#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/concept/assert.hpp>
// #include <boost/graph/graph_concepts.hpp>
// #include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "boost-graph-ndn-global-routing-helper.h"

#include <math.h>

NS_LOG_COMPONENT_DEFINE ("ndn.GlobalRoutingHelper");

using namespace std;
using namespace boost;

namespace ns3 {
namespace ndn {

void
GlobalRoutingHelper::Install (Ptr<Node> node)
{
  NS_LOG_LOGIC ("Node: " << node->GetId ());

  Ptr<L3Protocol> ndn = node->GetObject<L3Protocol> ();
  NS_ASSERT_MSG (ndn != 0, "Cannot install GlobalRoutingHelper before Ndn is installed on a node");

  Ptr<GlobalRouter> gr = node->GetObject<GlobalRouter> ();
  if (gr != 0)
    {
      NS_LOG_DEBUG ("GlobalRouter is already installed: " << gr);
      return; // already installed
    }

  gr = CreateObject<GlobalRouter> ();
  node->AggregateObject (gr);

  for (uint32_t faceId = 0; faceId < ndn->GetNFaces (); faceId++)
    {
      Ptr<NetDeviceFace> face = DynamicCast<NetDeviceFace> (ndn->GetFace (faceId));
      if (face == 0)
	{
	  NS_LOG_DEBUG ("Skipping non-netdevice face");
	  continue;
	}

      Ptr<NetDevice> nd = face->GetNetDevice ();
      if (nd == 0)
	{
	  NS_LOG_DEBUG ("Not a NetDevice associated with NetDeviceFace");
	  continue;
	}

      Ptr<Channel> ch = nd->GetChannel ();

      if (ch == 0)
	{
	  NS_LOG_DEBUG ("Channel is not associated with NetDevice");
	  continue;
	}

      if (ch->GetNDevices () == 2) // e.g., point-to-point channel
	{
	  for (uint32_t deviceId = 0; deviceId < ch->GetNDevices (); deviceId ++)
	    {
	      Ptr<NetDevice> otherSide = ch->GetDevice (deviceId);
	      if (nd == otherSide) continue;

	      Ptr<Node> otherNode = otherSide->GetNode ();
	      NS_ASSERT (otherNode != 0);

	      Ptr<GlobalRouter> otherGr = otherNode->GetObject<GlobalRouter> ();
	      if (otherGr == 0)
		{
		  Install (otherNode);
		}
	      otherGr = otherNode->GetObject<GlobalRouter> ();
	      NS_ASSERT (otherGr != 0);
	      gr->AddIncidency (face, otherGr);
	    }
	}
      else
	{
	  Ptr<GlobalRouter> grChannel = ch->GetObject<GlobalRouter> ();
	  if (grChannel == 0)
	    {
	      Install (ch);
	    }
	  grChannel = ch->GetObject<GlobalRouter> ();

	  gr->AddIncidency (face, grChannel);
	}
    }
}

void
GlobalRoutingHelper::Install (Ptr<Channel> channel)
{
  NS_LOG_LOGIC ("Channel: " << channel->GetId ());

  Ptr<GlobalRouter> gr = channel->GetObject<GlobalRouter> ();
  if (gr != 0)
    return;

  gr = CreateObject<GlobalRouter> ();
  channel->AggregateObject (gr);

  for (uint32_t deviceId = 0; deviceId < channel->GetNDevices (); deviceId ++)
    {
      Ptr<NetDevice> dev = channel->GetDevice (deviceId);

      Ptr<Node> node = dev->GetNode ();
      NS_ASSERT (node != 0);

      Ptr<GlobalRouter> grOther = node->GetObject<GlobalRouter> ();
      if (grOther == 0)
	{
	  Install (node);
	}
      grOther = node->GetObject<GlobalRouter> ();
      NS_ASSERT (grOther != 0);

      gr->AddIncidency (0, grOther);
    }
}

void
GlobalRoutingHelper::Install (const NodeContainer &nodes)
{
  for (NodeContainer::Iterator node = nodes.Begin ();
       node != nodes.End ();
       node ++)
    {
      Install (*node);
    }
}

void
GlobalRoutingHelper::InstallAll ()
{
  Install (NodeContainer::GetGlobal ());
}


void
GlobalRoutingHelper::AddOrigin (const std::string &prefix, Ptr<Node> node)
{
  Ptr<GlobalRouter> gr = node->GetObject<GlobalRouter> ();
  NS_ASSERT_MSG (gr != 0,
		 "GlobalRouter is not installed on the node");

  Ptr<Name> name = Create<Name> (boost::lexical_cast<Name> (prefix));
  gr->AddLocalPrefix (name);
}

void
GlobalRoutingHelper::AddOrigins (const std::string &prefix, const NodeContainer &nodes)
{
  for (NodeContainer::Iterator node = nodes.Begin ();
       node != nodes.End ();
       node++)
    {
      AddOrigin (prefix, *node);
    }
}

void
GlobalRoutingHelper::AddOrigin (const std::string &prefix, const std::string &nodeName)
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  NS_ASSERT_MSG (node != 0, nodeName << "is not a Node");

  AddOrigin (prefix, node);
}

void
GlobalRoutingHelper::AddOriginsForAll ()
{
  for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node ++)
    {
      Ptr<GlobalRouter> gr = (*node)->GetObject<GlobalRouter> ();
      string name = Names::FindName (*node);

      if (gr != 0 && !name.empty ())
        {
          AddOrigin ("/"+name, *node);
        }
    }
}

void
GlobalRoutingHelper::CalculateRoutes (bool invalidatedRoutes/* = true*/)
{
  /**
   * Implementation of route calculation is heavily based on Boost Graph Library
   * See http://www.boost.org/doc/libs/1_49_0/libs/graph/doc/table_of_contents.html for more details
   */

  BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< NdnGlobalRouterGraph > ));

  NdnGlobalRouterGraph graph;
  // typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;

  // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
  // Other algorithms should be faster, but they need additional EdgeListGraph concept provided by the graph, which
  // is not obviously how implement in an efficient manner
  for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
    {
      Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
      if (source == 0)
	{
	  NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
	  continue;
	}

      DistancesMap    distances;

      dijkstra_shortest_paths (graph, source,
			       // predecessor_map (boost::ref(predecessors))
			       // .
			       distance_map (boost::ref(distances))
			       .
			       distance_inf (WeightInf)
			       .
			       distance_zero (WeightZero)
			       .
			       distance_compare (boost::WeightCompare ())
			       .
			       distance_combine (boost::WeightCombine ())
			       );

      // NS_LOG_DEBUG (predecessors.size () << ", " << distances.size ());

      Ptr<Fib>  fib  = source->GetObject<Fib> ();
      if (invalidatedRoutes)
        {
          fib->InvalidateAll ();
        }
      NS_ASSERT (fib != 0);

      NS_LOG_DEBUG ("Reachability from Node: " << source->GetObject<Node> ()->GetId ());
      for (DistancesMap::iterator i = distances.begin ();
	   i != distances.end ();
	   i++)
	{
	  if (i->first == source)
	    continue;
	  else
	    {
	      // cout << "  Node " << i->first->GetObject<Node> ()->GetId ();
	      if (i->second.get<0> () == 0)
		{
		  // cout << " is unreachable" << endl;
		}
	      else
		{
                  BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                    {
                      NS_LOG_DEBUG (" prefix " << prefix << " reachable via face " << *i->second.get<0> ()
                                    << " with distance " << i->second.get<1> ()
                                    << " with delay " << i->second.get<2> ());

                      Ptr<fib::Entry> entry = fib->Add (prefix, i->second.get<0> (), i->second.get<1> ());
                      entry->SetRealDelayToProducer (i->second.get<0> (), Seconds (i->second.get<2> ()));

                      Ptr<Limits> faceLimits = i->second.get<0> ()->GetObject<Limits> ();

                      Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                      if (fibLimits != 0)
                        {
                          // if it was created by the forwarding strategy via DidAddFibEntry event
                          fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 * i->second.get<2> () /*exact RTT*/);
                          NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                        2*i->second.get<2> () << "s (" << faceLimits->GetMaxRate () * 2 * i->second.get<2> () << ")");
                        }
                    }
		}
	    }
	}
    }
}

void
GlobalRoutingHelper::CalculateAllPossibleRoutes (bool invalidatedRoutes/* = true*/)
{
  /**
   * Implementation of route calculation is heavily based on Boost Graph Library
   * See http://www.boost.org/doc/libs/1_49_0/libs/graph/doc/table_of_contents.html for more details
   */

  BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< NdnGlobalRouterGraph > ));

  NdnGlobalRouterGraph graph;
  // typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;

  // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
  // Other algorithms should be faster, but they need additional EdgeListGraph concept provided by the graph, which
  // is not obviously how implement in an efficient manner
  for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
    {
      Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
      if (source == 0)
	{
	  NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
	  continue;
	}

      Ptr<Fib>  fib  = source->GetObject<Fib> ();
      if (invalidatedRoutes)
        {
          fib->InvalidateAll ();
        }
      NS_ASSERT (fib != 0);

      NS_LOG_DEBUG ("===========");
      NS_LOG_DEBUG ("Reachability from Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");

      Ptr<L3Protocol> l3 = source->GetObject<L3Protocol> ();
      NS_ASSERT (l3 != 0);

      // remember interface statuses
      std::vector<uint16_t> originalMetric (l3->GetNFaces ());
      for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
        {
          originalMetric[faceId] = l3->GetFace (faceId)->GetMetric ();
          l3->GetFace (faceId)->SetMetric (std::numeric_limits<uint16_t>::max ()-1); // value std::numeric_limits<uint16_t>::max () MUST NOT be used (reserved)
        }

      for (uint32_t enabledFaceId = 0; enabledFaceId < l3->GetNFaces (); enabledFaceId++)
        {
          if (DynamicCast<ndn::NetDeviceFace> (l3->GetFace (enabledFaceId)) == 0)
            continue;

          // enabling only faceId
          l3->GetFace (enabledFaceId)->SetMetric (originalMetric[enabledFaceId]);

          DistancesMap    distances;

          NS_LOG_DEBUG ("-----------");

          dijkstra_shortest_paths (graph, source,
                                   // predecessor_map (boost::ref(predecessors))
                                   // .
                                   distance_map (boost::ref(distances))
                                   .
                                   distance_inf (WeightInf)
                                   .
                                   distance_zero (WeightZero)
                                   .
                                   distance_compare (boost::WeightCompare ())
                                   .
                                   distance_combine (boost::WeightCombine ())
                                   );

          // NS_LOG_DEBUG (predecessors.size () << ", " << distances.size ());

          for (DistancesMap::iterator i = distances.begin ();
               i != distances.end ();
               i++)
            {
              if (i->first == source)
                continue;
              else
                {
                  // cout << "  Node " << i->first->GetObject<Node> ()->GetId ();
                  if (i->second.get<0> () == 0)
                    {
                      // cout << " is unreachable" << endl;
                    }
                  else
                    {
                      BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                        {
                          NS_LOG_DEBUG (" prefix " << *prefix << " reachable via face " << *i->second.get<0> ()
                                        << " with distance " << i->second.get<1> ()
                                        << " with delay " << i->second.get<2> ());

                          if (i->second.get<0> ()->GetMetric () == std::numeric_limits<uint16_t>::max ()-1)
                            continue;

                          Ptr<fib::Entry> entry = fib->Add (prefix, i->second.get<0> (), i->second.get<1> ());
                          entry->SetRealDelayToProducer (i->second.get<0> (), Seconds (i->second.get<2> ()));

                          Ptr<Limits> faceLimits = i->second.get<0> ()->GetObject<Limits> ();

                          Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                          if (fibLimits != 0)
                            {
                              // if it was created by the forwarding strategy via DidAddFibEntry event
                              fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 * i->second.get<2> () /*exact RTT*/);
                              NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                            2*i->second.get<2> () << "s (" << faceLimits->GetMaxRate () * 2 * i->second.get<2> () << ")");
                            }
                        }
                    }
                }
            }

          // disabling the face again
          l3->GetFace (enabledFaceId)->SetMetric (std::numeric_limits<uint16_t>::max ()-1);
        }

      // recover original interface statuses
      for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
        {
          l3->GetFace (faceId)->SetMetric (originalMetric[faceId]);
        }
    }
}

// ZhangYu no common link multi-path routing
std::vector <std::vector<uint16_t> >  originalMetric;  	//注意这里的> >之间要有空格，否则error: ‘>>’ should be ‘> >’ within a nested template argument list
/*
 * 在这个函数中，执行的是把图中所有的节点端口的Metric备份到 originalMetric或者从其中恢复。在备份的过程中有清空节点fib的语句。
 * 这个清空fib的语句似乎不应该放在备份和恢复函数中，但是因为是从代码直接挪出来的，所以暂时这样 2014-2-1
 */
void
    BackupRestoreOrignalMetrics(const std::string action)
    {
        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
        BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
        NdnGlobalRouterGraph graph;
        typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;

        //保存所有边的OriginalMetric，因为找不到graph的一个边的集合，所以还是根据节点来遍历
        originalMetric.resize(NodeList::GetNNodes());

        for(NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
        {
            int nodeId=(*node)->GetId();
            Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
            if (source == 0)
            {
                NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
                continue;
            }

            Ptr<Fib>  fib  = source->GetObject<Fib> ();   //只有这里获取fib，后面添加 Entry
            NS_ASSERT (fib != 0);
            if (action=="Backup&Initial")
            {
			//NS_LOG_DEBUG("ZhangYu ==================================================================================================");
			fib->InvalidateAll ();  //2014-1-8，这一句最终调用的是fib-entry.cc中的，把一个节点的所有端口都设置为如：dev[2]=net(1,2-5)(65535,r,1) 后面的65535，r（表示RED）
			//因为没有只在Backup中执行，在每个节点进行多路径计算时，要恢复一下 metric，如果也运行上面InvaliateAll，会导致前面节点计算后添加的fib entry变为 65535,r。从而出错
            }
            Ptr<L3Protocol> l3 = source->GetObject<L3Protocol> ();
            NS_ASSERT (l3 != 0);

            // remember interface statuses
            originalMetric[nodeId].resize(l3->GetNFaces ());

            for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
            {
                if (action=="Restore")
                {
                    l3->GetFace (faceId)->SetMetric (originalMetric[nodeId][faceId]); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
                    //NS_LOG_DEBUG("ZhangYu 2014-1-2 ===Restore====faceId: " <<faceId << "  Metric: "<< l3->GetFace (faceId)->GetMetric () );
                }
                else if ((action=="Backup")||(action=="Backup&Initial"))
                {
                	originalMetric[nodeId][faceId] = l3->GetFace (faceId)->GetMetric ();
                    //NS_LOG_DEBUG("ZhangYu 2014-1-2 =========================================nodeId: " <<nodeId << "    faceId: " <<faceId << "  Metric: "<< originalMetric[nodeId][faceId] );
                    //NS_LOG_DEBUG("ZhangYu ===============================================" << l3->GetFace (faceId)->GetInstanceTypeId());

                    }
                else
                	NS_LOG_DEBUG("ZhangYu  input a wrong action string for function BackupRestoreOriginalMetrics");
            }

        }
    }

void
    GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes(Ptr<Node> srcNode, Ptr<Node> desNode,Ptr<Name> &prefix1)
    {
        uint32_t  multipathNumber=3;    //共计算几条多路径

        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
        BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
        NdnGlobalRouterGraph graph;
        typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;

        BackupRestoreOrignalMetrics("Backup");

        Ptr<GlobalRouter> source = (srcNode)->GetObject<GlobalRouter>();


        //NS_LOG_DEBUG("ZhangYu 2014-1-1 is consumer node Id: " << (*node)->GetId() <<" " << (appTypeStr.find("Consumer")) <<"'  "<< appTypeStr);
        //NS_LOG_DEBUG ("===== Reachability from source Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");


        //计算包含这个consumer的节点到其他节点的最短路
        for(uint32_t pathIndex=0; pathIndex<multipathNumber;pathIndex++)
        {
            DistancesMap    distances;
            PredecessorsMap predecessors;

            dijkstra_shortest_paths (graph, source,
                                     predecessor_map (boost::ref(predecessors))
                                     .
                                     distance_map (boost::ref(distances))
                                     .
                                     distance_inf (WeightInf)
                                     .
                                     distance_zero (WeightZero)
                                     .
                                     distance_compare (boost::WeightCompare ())
                                     .
                                     distance_combine (boost::ZYWeightCombine ())
                                     );

            DistancesMap::iterator des=distances.find(desNode->GetObject<GlobalRouter>());
            BOOST_FOREACH (const Ptr<const Name> &prefix, des->first->GetLocalPrefixes ())
            {
                Ptr<GlobalRouter> curNode =des->first ;
                Ptr<GlobalRouter> preNode;
                NS_LOG_DEBUG("ZhangYu 2014-1-7 producer Node: " << curNode->GetObject<Node>()->GetId() << std::endl);

                while (curNode!=source) //回溯到源节点，添加fib，修改链路
                {
                    preNode=predecessors[curNode];
                    Ptr<Fib> fib  = preNode->GetObject<Fib> ();   //这里获取fib，后面添加 Entry
                    NS_ASSERT (fib != 0);   //https://www.nsnam.org/doxygen/group__assert.html#details

                    if(uint16_t( des->second.get<1>()-distances[curNode].get<1> ())>= std::numeric_limits<uint16_t>::max()-1)
                    {
                        std::cout << "ZhangYu 2014-1-8 我认为不应该出现这种情况，出现了是有逻辑错误" << std::endl << std::endl;
                        continue;
                    }
                    Ptr<fib::Entry> entry = fib->Add (prefix, distances[curNode].get<0> (),  des->second.get<1>()-distances[preNode].get<1> ());
                    NS_LOG_DEBUG("ZhangYu 2014-1-8 *entry: " << *entry);

                    entry->SetRealDelayToProducer (distances[curNode].get<0> (), Seconds (des->second.get<2>()-distances[preNode].get<2>()));

                    Ptr<Limits> faceLimits = distances[curNode].get<0> ()->GetObject<Limits> ();
                    Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                    if (fibLimits != 0)
                    {
                        // if it was created by the forwarding strategy via DidAddFibEntry event
                        fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 *  (des->second.get<2>()-distances[preNode].get<2>())/*exact RTT*/);
                        NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                      2* (des->second.get<2>()-distances[preNode].get<2>()) << "s (" << faceLimits->GetMaxRate () * 2 *  (des->second.get<2>()-distances[preNode].get<2>())<< ")");
                    }

                    //前面执行完了回溯路径，添加fib，后面的是把这条路径上的Link设置为不可用
                    //更改边的代价时，可以参考CaculateAllPossibleRoutes中的l3->GetFace (faceId),这里更简单的是使用distances[curNode].get<0>()，一样的类型
                    distances[curNode].get<0>()->SetMetric(std::numeric_limits<int16_t>::max ()-1); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)

                    curNode=preNode;
                }
            }

        }
        //恢复originalMetric
        BackupRestoreOrignalMetrics("Restore");
        NS_LOG_DEBUG("ZhangYu 2014-1-6 =========================================================================end of CalculateNoCommLinkMultiPathRoutes");
    }



void
    GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes()
    {
        uint32_t  multipathNumber=2;    //共计算几条多路径

        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
        BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
        NdnGlobalRouterGraph graph;
        typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;

        BackupRestoreOrignalMetrics("Backup&Initial");
        /*ZhangYu 2013-12-31 上面的语句得到的是 ns3::ndn::ConsumerCbr, ns3::Application   ns3::ndn::Producer ns3::Application，为了实现noComLinkMultiPath，要选择consumer节点才进行最短路径的计算
         * 计算出来后一次为所有Path上的节点都添加Fib，这样可以省去为无关的节点也计算最短路，计算一次才为当前计算的节点添加Fib。
         * 为了选择consumer节点，一种方式是在global-routing中(*node)->GetApplication(appId)->GetInstanceTypeId()判断节点的类型，根据字符串开头是consumer的，虽然可以考虑给Node再增加一个属性用来
         * 区分是consumer，这样可以挑出是consumer的节点来进行计算。现有的路由计算中，对source缩小范围，只对属于consumer的source进行计算，所以设置一个属性，不考虑producer，也暂时不考虑一个节点装在了多个consumer的情况
         * 但是增加一个node的属性，需要直接修改NS3代码中的node.cc，影响可能大，所以放弃，只是靠
         */
        for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
        {
           Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
            if (source == 0)
            {
                NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
                continue;
            }
            //开始计算最短路
            for(uint32_t appId=0; appId<(*node)->GetNApplications();appId++)
            {
                std::string appTypeStr= (*node)->GetApplication(appId)->GetInstanceTypeId().GetName();
                if(std::string::npos!= appTypeStr.find("Consumer"))
                {
                   	//NS_LOG_DEBUG("ZhangYu 2014-1-1 is consumer node Id: " << (*node)->GetId() <<" " << (appTypeStr.find("Consumer")) <<"'  "<< appTypeStr);
                    NS_LOG_DEBUG ("===== Reachability from source Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");


                    //计算包含这个consumer的节点到其他节点的最短路
                    for(uint32_t pathIndex=0; pathIndex<multipathNumber;pathIndex++)
                    {
                        DistancesMap    distances;
                        PredecessorsMap predecessors;
                        dijkstra_shortest_paths (graph, source,
                                                 predecessor_map (boost::ref(predecessors))
                                                 .
                                                 distance_map (boost::ref(distances))
                                                 .
                                                 distance_inf (WeightInf)
                                                 .
                                                 distance_zero (WeightZero)
                                                 .
                                                 distance_compare (boost::WeightCompare ())
                                                 .
                                                 distance_combine (boost::ZYWeightCombine ())
                                                 );
                        NS_LOG_DEBUG("ZhangYu 2014-2-7 pathIndex: " << pathIndex << endl);
                        for(PredecessorsMap::iterator i=predecessors.begin();i!=predecessors.end();i++)
                        {
                            //NS_LOG_DEBUG("ZhangYu 2013-5-21 predecessors node: " << i->first->GetObject<Node>()->GetId()  <<"  ParentNode: " <<i->second->GetObject<Node>()->GetId());
                        }

                        for (DistancesMap::iterator i = distances.begin (); i != distances.end (); i++)
                        {
                            if (i->first == source)
                                continue;
                            else
                            {
                                if (i->second.get<0> () == 0)
                                {
                                    cout <<"  Node " << i->first->GetObject<Node> ()->GetId () << " is unreachable" << endl;
                                }
                                else
                                {
                                    NS_LOG_DEBUG("ZhangYu 2014-1-3, Node:" << i->first->GetObject<Node>()->GetId()<< "   face:" << *i->second.get<0>()<<"  with distance:" <<i->second.get<1>());

                                    //下面的语句使得为每个producer的节点的每个应用添加路由fibs，为0就不循环，一个节点有多个Apps时循环（这里循环执行有点冗余，因为步骤一样，只是prefix不同，但是为了代码清爽，就这样了）
                                    NS_LOG_DEBUG("ZhangYu 2014-2-7 i->first->GetLocalPrefixes.size(): " <<i->first->GetLocalPrefixes().size());

                                    BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                                    {
                                        Ptr<GlobalRouter> curNode =i->first ;
                                        Ptr<GlobalRouter> preNode;
                                        NS_LOG_DEBUG("ZhangYu 2014-1-7 producer Node: " << curNode->GetObject<Node>()->GetId() );

                                        while (curNode!=source)
                                        {
                                            preNode=predecessors[curNode];
                                            NS_LOG_DEBUG("ZhangYu  2014-1-5 prefix: " << *prefix << "  Node: " << preNode->GetObject<Node>()->GetId()
                                                         << "  reachable via face: " << *distances[curNode].get<0>()
                                                         << "  with distance: " << i->second.get<1>()-distances[preNode].get<1>()
                                                         << "  with delay " << distances[curNode].get<2>());


                                            Ptr<Fib> fib  = preNode->GetObject<Fib> ();   //这里获取fib，后面添加 Entry
                                            //ZhangYu 2014-1-6，下面的这一句使得每个节点的所有出口都变成值最大，导致传播消息时出错
                                            //fib->InvalidateAll ();
                                            NS_ASSERT (fib != 0);   //2014-1-9现在还不清楚是否可以去掉这句

                                            if(uint16_t( i->second.get<1>()-distances[curNode].get<1> ())== std::numeric_limits<uint16_t>::max()-1)
                                              {
                                                std::cout << "ZhangYu 2014-1-8 我认为不应该出现这种情况，出现了是有逻辑错误" << std::endl << std::endl;
                                              continue;
                                              }
                                        	//Ptr<Name> temp=Create<Name> (boost::lexical_cast<string>(*prefix)+"/"+boost::lexical_cast<string>(pathIndex));
                                            //const Ptr<const Name> temp=Create<Name> (boost::lexical_cast<string>(*prefix));
                                            //NS_LOG_DEBUG("ZhangYu 2014-1-8 temp: " << *temp);
                                            //Ptr<fib::Entry> entry = fib->Add (temp, distances[curNode].get<0> (),  i->second.get<1>()-distances[preNode].get<1> ());
                                            //NS_LOG_DEBUG("ZhangYu 2014-3-22 distances[curNode] : " << i->second.get<1>() << "   distances[preNode].get<1>(): " << distances[preNode].get<1>());
                                            if(i->second.get<1>()-distances[preNode].get<1> ()<std::numeric_limits<uint16_t>::max())
                                            {
                                            	Ptr<fib::Entry> entry = fib->Add (prefix, distances[curNode].get<0> (),  i->second.get<1>()-distances[preNode].get<1> ());
                                                entry->SetRealDelayToProducer (distances[curNode].get<0> (), Seconds (i->second.get<2>()-distances[preNode].get<2>()));

                                                Ptr<Limits> faceLimits = distances[curNode].get<0> ()->GetObject<Limits> ();
                                                Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                                                if (fibLimits != 0)
                                                {
                                                    // if it was created by the forwarding strategy via DidAddFibEntry event
                                                    fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 *  (i->second.get<2>()-distances[preNode].get<2>())/*exact RTT*/);
                                                    NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                                                  2* (i->second.get<2>()-distances[preNode].get<2>()) << "s (" << faceLimits->GetMaxRate () * 2 *  (i->second.get<2>()-distances[preNode].get<2>())<< ")");
                                                }
                                                NS_LOG_DEBUG("ZhangYu 2014-2-9 *entry: " << *entry);
                                            }
                                            else
                                            	NS_LOG_DEBUG("ZhangYu 2014-3-22 didnot add fib, because greater than uint16::max");

                                            //前面执行完了回溯路径，添加fib，后面的是把这条路径上的Link设置为不可用
                                            //更改边的代价时，可以参考CaculateAllPossibleRoutes中的l3->GetFace (faceId),这里更简单的是使用distances[curNode].get<0>()，一样的类型
                                            //NS_LOG_DEBUG("ZhangYu 2014-1-9 distances[curNode].get<0>()->GetInstanceTypeId()=====" << distances[curNode].get<0>()->GetInstanceTypeId());
                                            distances[curNode].get<0>()->SetMetric(std::numeric_limits<uint16_t>::max ()); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
                                            //distances[curNode].get<0>()->SetMetric(2000); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)

                                            curNode=preNode;
                                        }
                                        std::cout << "ZhangYu 2014-3-15  predecessors: " << preNode->GetId() << "   source: " << source->GetId() << std::endl;
                                    }
                                }
                            }
                        }

                    }

                    //恢复originalMetric
                    BackupRestoreOrignalMetrics("Restore");
                }
            }
        }
        NS_LOG_DEBUG("ZhangYu 2014-1-6 -end of CalculateNoCommLinkMultiPathRoutes");
    }

//----ZhangYu

} // namespace ndn
} // namespace ns3
