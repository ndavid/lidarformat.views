/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime. 


Homepage: 

	http://code.google.com/p/lidarformat
	
Copyright:
	
	Institut Geographique National & CEMAGREF (2009)

Author: 

	Adrien Chauve
	
Contributors:

	Nicolas David, Olivier Tournaire
	
	

    LidarFormat is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LidarFormat is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public 
    License along with LidarFormat.  If not, see <http://www.gnu.org/licenses/>.
 
***********************************************************************/

#ifndef LIDARDATAVIEW_HPP_
#define LIDARDATAVIEW_HPP_

#include "LidarFormat/LidarDataContainer.h"
#include <boost/iterator/permutation_iterator.hpp>
using namespace Lidar;

template<typename AttType>
class LidarDataAttView{

	public :
	 	typedef AttViewIterator<AttType> iterator;
	 	LidarDataAttView(boost::shared_ptr<LidarDataContainer> data, unsigned int offset,unsigned int stride) :
	 		m_data_ptr(data),
	 		m_att_offset(offset),
	 		m_att_stride(stride) {}
		iterator begin()
			{
				char * raw_begin_att=m_data_ptr->rawData() + m_att_offset;
				return iterator(raw_begin_att, m_att_stride);
			}
		iterator end()
			{
				int size=m_data_ptr->size();
				char * raw_end_data=m_data_ptr->rawData() + m_att_offset+m_att_stride*(size);
				return iterator(raw_end_data, m_att_stride);
			}

	private :
		boost::shared_ptr<LidarDataContainer> m_data_ptr;
		unsigned int m_att_offset;
		unsigned int m_att_stride;
};

//*******************************************************
// Multiple value / single type  proxy view .
//    examples : xy view, xyz view, xz view etc..
//******************************************************
template<typename AttType, int dim>
struct MakeViewProxyIterator
{
	AttViewProxyIterator<AttType, dim>  make(char*, unsigned int *){return AttViewProxyIterator<AttType, dim>();}
};

template<typename AttType>
struct MakeViewProxyIterator<AttType,1>
{
	AttViewProxyIterator<AttType, 1>
	make(char* raw_data,const unsigned int stride, const unsigned int m_att_offsets[1] ) const{
		return AttViewProxyIterator<AttType, 1>(raw_data, stride,
				m_att_offsets[0]
		);
	}
};

template<typename AttType>
struct MakeViewProxyIterator<AttType,2>
{
	AttViewProxyIterator<AttType, 2>
	make(char* raw_data, const unsigned int stride, const unsigned int m_att_offsets[2] ) const{
		return AttViewProxyIterator<AttType, 2>(raw_data, stride,
		m_att_offsets[0],
		m_att_offsets[1]
		);
		}
};

template<typename AttType>
struct MakeViewProxyIterator<AttType,3>
{
	AttViewProxyIterator<AttType, 3>
	make(char* raw_data,  const unsigned int stride, const unsigned int m_att_offsets[3]) const {
		return AttViewProxyIterator<AttType, 3>(raw_data, stride,
				m_att_offsets[0],
				m_att_offsets[1],
				m_att_offsets[2]
	    		);
		}
};
template<typename AttType, int dim>
class LidarDataAttProxyView{

	public :
	 	typedef AttViewProxyIterator<AttType,dim> iterator;
	 	LidarDataAttProxyView(boost::shared_ptr<LidarDataContainer> data, unsigned int stride, unsigned int offset0=0,
	    		unsigned int offset1=0,
	    		unsigned int offset2=0,
	    		unsigned int offset3=0
	    		)
	      : m_data_ptr(data),m_att_stride(stride)
		{
	    	  if (dim >= 1) m_att_offsets[0] = offset0;
	    	  if (dim >= 2) m_att_offsets[1] = offset1;
	    	   if (dim >= 3) m_att_offsets[2] = offset2;
	    	   if (dim >= 3) m_att_offsets[3] = offset3;
		}
		iterator begin()
			{
				char * raw_begin_att=m_data_ptr->rawData();
				return make_iterator.make(raw_begin_att, m_att_stride,m_att_offsets);
			}
		iterator end()
			{
				int size=m_data_ptr->size();
				char * raw_end_data=m_data_ptr->rawData()+m_att_stride*(size);
				return make_iterator.make(raw_end_data, m_att_stride, m_att_offsets);
			}

	private :
		boost::shared_ptr<LidarDataContainer> m_data_ptr;
		unsigned int m_att_stride;
		unsigned int m_att_offsets[dim];
		MakeViewProxyIterator<AttType, dim> make_iterator;
};

//*******************************************************
// Multiple value / single type  proxy view .
//    examples : xy view, xyz view, xz view etc..
//******************************************************
template<typename AttType, int dim>
class LidarDataAttProxyIndexView{

	public :
	 	typedef AttViewProxyIterator<AttType,dim> element_iterator;
	 	typedef std::vector<int>::iterator index_iterator;
	 	typedef boost::permutation_iterator< element_iterator, index_iterator > iterator;

	 	LidarDataAttProxyIndexView(boost::shared_ptr<LidarDataContainer> data,boost::shared_ptr<std::vector<int> > index, unsigned int stride, unsigned int offset0=0,
	    		unsigned int offset1=0,
	    		unsigned int offset2=0,
	    		unsigned int offset3=0
	    		)
	      : m_data_ptr(data),m_index_ptr(index),m_att_stride(stride)
		{
	    	  if (dim >= 1) m_att_offsets[0] = offset0;
	    	  if (dim >= 2) m_att_offsets[1] = offset1;
	    	   if (dim >= 3) m_att_offsets[2] = offset2;
	    	   if (dim >= 3) m_att_offsets[3] = offset3;
		}
		iterator begin()
			{
				char * raw_begin_att=m_data_ptr->rawData();
				//return make_iterator.make(raw_begin_att, m_att_stride,m_att_offsets);
				return make_permutation_iterator( make_iterator.make(raw_begin_att, m_att_stride,m_att_offsets), m_index_ptr->begin() );
				//permutation_type it = begin;
				//permutation_type end = make_permutation_iterator( elements.begin(), indices.end() );

			}
		iterator end()
			{
			char * raw_begin_att=m_data_ptr->rawData();
							//return make_iterator.make(raw_begin_att, m_att_stride,m_att_offsets);
			return make_permutation_iterator( make_iterator.make(raw_begin_att, m_att_stride,m_att_offsets), m_index_ptr->end() );
			}

	private :
		boost::shared_ptr<LidarDataContainer> m_data_ptr;
		boost::shared_ptr<std::vector<int> > m_index_ptr;
		unsigned int m_att_stride;
		unsigned int m_att_offsets[dim];
		MakeViewProxyIterator<AttType, dim> make_iterator;
};

#endif /* LIDARDATAVIEW_HPP_ */
