/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 University of California, Los Angeles
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


#ifndef BOOST_GRAPH_NDN_GLOBAL_ROUTING_HELPER_H
#define BOOST_GRAPH_NDN_GLOBAL_ROUTING_HELPER_H

/// @cond include_hidden

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/ref.hpp>

#include "ns3/ndn-face.h"
#include "ns3/ndn-limits.h"
#include "ns3/node-list.h"
#include "ns3/channel-list.h"
#include "../model/ndn-global-router.h"
#include <list>
#include <map>

namespace boost {

class NdnGlobalRouterGraph
{
public:
  typedef ns3::Ptr< ns3::ndn::GlobalRouter > Vertice;
  typedef uint16_t edge_property_type;
  typedef uint32_t vertex_property_type;
  
  NdnGlobalRouterGraph ()
  {
    for (ns3::NodeList::Iterator node = ns3::NodeList::Begin (); node != ns3::NodeList::End (); node++)
      {
        ns3::Ptr<ns3::ndn::GlobalRouter> gr = (*node)->GetObject<ns3::ndn::GlobalRouter> ();
	if (gr != 0)
	  m_vertices.push_back (gr);
      }

    for (ns3::ChannelList::Iterator channel = ns3::ChannelList::Begin (); channel != ns3::ChannelList::End (); channel++)
      {
        ns3::Ptr<ns3::ndn::GlobalRouter> gr = (*channel)->GetObject<ns3::ndn::GlobalRouter> ();
	if (gr != 0)
	  m_vertices.push_back (gr);
      }
  }

  const std::list< Vertice > &
  GetVertices () const
  {
    return m_vertices;
  }
  
public:
  std::list< Vertice > m_vertices;
};


class ndn_global_router_graph_category :
    public virtual vertex_list_graph_tag,
    public virtual incidence_graph_tag
{
};


template<>
struct graph_traits< NdnGlobalRouterGraph >
{
  // Graph concept
  typedef NdnGlobalRouterGraph::Vertice vertex_descriptor;
  typedef ns3::ndn::GlobalRouter::Incidency edge_descriptor;
  typedef directed_tag directed_category;
  typedef disallow_parallel_edge_tag edge_parallel_category;
  typedef ndn_global_router_graph_category traversal_category;

  // VertexList concept
  typedef std::list< vertex_descriptor >::const_iterator vertex_iterator;
  typedef size_t vertices_size_type;

  // AdjacencyGraph concept
  typedef ns3::ndn::GlobalRouter::IncidencyList::iterator out_edge_iterator;
  typedef size_t degree_size_type;

  // typedef size_t edges_size_type;
};

} // namespace boost

namespace boost
{

inline
graph_traits< NdnGlobalRouterGraph >::vertex_descriptor
source(
       graph_traits< NdnGlobalRouterGraph >::edge_descriptor e,
       const NdnGlobalRouterGraph& g)
{
  return e.get<0> ();
}

inline
graph_traits< NdnGlobalRouterGraph >::vertex_descriptor
target(
       graph_traits< NdnGlobalRouterGraph >::edge_descriptor e,
       const NdnGlobalRouterGraph& g)
{
  return e.get<2> ();
}

inline
std::pair< graph_traits< NdnGlobalRouterGraph >::vertex_iterator,
	   graph_traits< NdnGlobalRouterGraph >::vertex_iterator >
vertices (const NdnGlobalRouterGraph&g)
{
  return make_pair (g.GetVertices ().begin (), g.GetVertices ().end ());
}

inline
graph_traits< NdnGlobalRouterGraph >::vertices_size_type
num_vertices(const NdnGlobalRouterGraph &g)
{
  return g.GetVertices ().size ();
}
  

inline
std::pair< graph_traits< NdnGlobalRouterGraph >::out_edge_iterator,
	   graph_traits< NdnGlobalRouterGraph >::out_edge_iterator >  
out_edges(
	  graph_traits< NdnGlobalRouterGraph >::vertex_descriptor u, 
	  const NdnGlobalRouterGraph& g)
{
  return std::make_pair(u->GetIncidencies ().begin (),
			u->GetIncidencies ().end ());
}

inline
graph_traits< NdnGlobalRouterGraph >::degree_size_type
out_degree(
	  graph_traits< NdnGlobalRouterGraph >::vertex_descriptor u, 
	  const NdnGlobalRouterGraph& g)
{
  return u->GetIncidencies ().size ();
}


//////////////////////////////////////////////////////////////
// Property maps

struct EdgeWeights
{
  EdgeWeights (const NdnGlobalRouterGraph &graph)
  : m_graph (graph)
  { 
  }

private:
  const NdnGlobalRouterGraph &m_graph;
};


struct VertexIds
{
  VertexIds (const NdnGlobalRouterGraph &graph)
  : m_graph (graph)
  { 
  }

private:
  const NdnGlobalRouterGraph &m_graph;
};

template<>
struct property_map< NdnGlobalRouterGraph, edge_weight_t >
{
  typedef const EdgeWeights const_type;
  typedef EdgeWeights type;
};

template<>
struct property_map< NdnGlobalRouterGraph, vertex_index_t >
{
  typedef const VertexIds const_type;
  typedef VertexIds type;
};


template<>
struct property_traits< EdgeWeights >
{
  // Metric property map
  typedef tuple< ns3::Ptr<ns3::ndn::Face>, uint16_t, double > value_type;
  typedef tuple< ns3::Ptr<ns3::ndn::Face>, uint16_t, double > reference;
  typedef ns3::ndn::GlobalRouter::Incidency key_type;
  typedef readable_property_map_tag category;
};

const property_traits< EdgeWeights >::value_type WeightZero (0, 0, 0.0);
const property_traits< EdgeWeights >::value_type WeightInf (0, std::numeric_limits<uint16_t>::max (), 0.0);

struct WeightCompare :
    public std::binary_function<property_traits< EdgeWeights >::reference,
                                property_traits< EdgeWeights >::reference,
                                bool>
{
  bool
  operator () (tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > a,
               tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > b) const
  {
    return a.get<1> () < b.get<1> ();
  }

  bool
  operator () (property_traits< EdgeWeights >::reference a,
               uint32_t b) const
  {
    return a.get<1> () < b;
  }
  
  bool
  operator () (uint32_t a,
               uint32_t b) const
  {
    return a < b;
  }

};

struct WeightCombine :
    public std::binary_function<uint32_t,
                                property_traits< EdgeWeights >::reference,
                                uint32_t>
{
  uint32_t
  operator () (uint32_t a, property_traits< EdgeWeights >::reference b) const
  {
    return a + b.get<1> ();
  }

  tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double >
  operator () (tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > a,
               property_traits< EdgeWeights >::reference b) const
  {
    if (a.get<0> () == 0)
      return make_tuple (b.get<0> (), a.get<1> () + b.get<1> (), a.get<2> () + b.get<2> ());
    else
      return make_tuple (a.get<0> (), a.get<1> () + b.get<1> (), a.get<2> () + b.get<2> ());
  }
};

/*
 * 2014-1-4，因为在原始的代码中，当节点0为源节点调用dijkstra算法时，例如节点4，能够得到如下
 * ZhangYu 2014-1-3, Node:4   face:dev[0]=net(0,0-1)  with distance:2，含义是到从节点0到节点4的距离是2，从节点0的出口faces是dev[0]=net(0,0-1)。
 * 而我在多路径计算中，希望能得到的是从节点4到节点0的所有的faces，即使麻烦，那也至少是4-1的face，而不是0-1
 * 如果有4-1，那么就可以通过追溯parent node而得到整条path
 */
struct ZYWeightCombine :
    public std::binary_function<uint32_t,
                                property_traits< EdgeWeights >::reference,
                                uint32_t>
{
  uint32_t
  operator () (uint32_t a, property_traits< EdgeWeights >::reference b) const
  {
    return a + b.get<1> ();
  }

  tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double >
  operator () (tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > a,
               property_traits< EdgeWeights >::reference b) const
  {
	/*b已经是<face,uint,double>的元组了，这里只是告诉使用这样的距离计算方法。b的产生实在本文件下面的的get(weightmap,edge)的定义中
	 * get(weightmap,edge）是在dijkstra_shortest_path中调用了 relax， relax中有 const W& w_e = get(w, e);所以除非修改get(w,e)
	 * 否则任何在dijkstra外部的对b的修改是无效的。
	 * 2013-5-15
	 */
	/*
	* ZhangYu 2014-3-23 因为multipath产生了错误，所以要理解compare和combine到底干了什么
	* a<1>是当前节点的标号，b<1>是当前边的代价，那么返回的应该是新的节点的代价
	*/
	//std::cout << "ZhangYu 2014-3-21 a<1>" << a.get<1>() << "   b<1>" << b.get<1>() << "   a<2> "<< a.get<2>() << "  b<2>" << b.get<2>() << std::endl;
	//std::cout << "ZhangYu 2014-3-212： a<1>+b<1>: " << a.get<1>() +b.get<1>() << std::endl;
    if (a.get<0> () == 0)
    {
      return make_tuple (b.get<0> (), a.get<1> () + b.get<1> (), a.get<2> () + b.get<2> ());
    }
    else
      {
      //std::cout<< "ZhangYu ***********************************************************************************************************  " << *(b.get<0>()) <<  std::endl;
      return make_tuple (b.get<0> (), a.get<1> () + b.get<1> (), a.get<2> () + b.get<2> ());
      }
  }

  //ZhangYu 2014-1-4添加，为了多路径计算能够追溯得到整条路径的faces。为了使得源代码的改动最小，所以在返回tuple的最后添加b.get<0>()，

};



template<>
struct property_traits< VertexIds >
{
  // Metric property map
  typedef uint32_t value_type;
  typedef uint32_t reference;
  typedef ns3::Ptr< ns3::ndn::GlobalRouter > key_type;
  typedef readable_property_map_tag category;
};


inline EdgeWeights
get(edge_weight_t,
    const NdnGlobalRouterGraph &g)
{
  return EdgeWeights (g);
}


inline VertexIds
get(vertex_index_t,
    const NdnGlobalRouterGraph &g)
{
  return VertexIds (g);
}

template<class M, class K, class V>
inline void
put (reference_wrapper< M > mapp,
     K a, V p)
{
  mapp.get ()[a] = p;
}

// void
// put (cref< std::map< ns3::Ptr<ns3::ndn::GlobalRouter>, ns3::Ptr<ns3::ndn::GlobalRouter> > > map,

inline uint32_t
get (const boost::VertexIds&, ns3::Ptr<ns3::ndn::GlobalRouter> &gr)
{
  return gr->GetId ();
}

inline property_traits< EdgeWeights >::reference
get(const boost::EdgeWeights&, ns3::ndn::GlobalRouter::Incidency &edge)
{
  if (edge.get<1> () == 0)
    return property_traits< EdgeWeights >::reference (0, 0, 0.0);
  else
    {
      ns3::Ptr<ns3::ndn::Limits> limits = edge.get<1> ()->GetObject<ns3::ndn::Limits> ();
      double delay = 0.0;
      if (limits != 0) // valid limits object
        {
          delay = limits->GetLinkDelay ();
        }
      return property_traits< EdgeWeights >::reference (edge.get<1> (), edge.get<1> ()->GetMetric (), delay);
    }
}

struct PredecessorsMap :
    public std::map< ns3::Ptr< ns3::ndn::GlobalRouter >, ns3::Ptr< ns3::ndn::GlobalRouter > >
{
};

template<>
struct property_traits< reference_wrapper<PredecessorsMap> >
{
  // Metric property map
  typedef ns3::Ptr< ns3::ndn::GlobalRouter > value_type;
  typedef ns3::Ptr< ns3::ndn::GlobalRouter > reference;
  typedef ns3::Ptr< ns3::ndn::GlobalRouter > key_type;
  typedef read_write_property_map_tag category;
};


struct DistancesMap :
  public std::map< ns3::Ptr< ns3::ndn::GlobalRouter >, tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > >
{
};

template<>
struct property_traits< reference_wrapper<DistancesMap> >
{
  // Metric property map
  typedef tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > value_type;
  typedef tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > reference;
  typedef ns3::Ptr< ns3::ndn::GlobalRouter > key_type;
  typedef read_write_property_map_tag category;
};

inline tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double >
get (DistancesMap &map, ns3::Ptr<ns3::ndn::GlobalRouter> key)
{
  boost::DistancesMap::iterator i = map.find (key);
  if (i == map.end ())
    return tuple< ns3::Ptr<ns3::ndn::Face>, uint32_t, double > (0, std::numeric_limits<uint32_t>::max (), 0.0);
  else
    return i->second;
}

} // namespace boost

/// @endcond

#endif // BOOST_GRAPH_NDN_GLOBAL_ROUTING_HELPER_H
