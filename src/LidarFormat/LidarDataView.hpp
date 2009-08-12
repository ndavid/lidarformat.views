/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point
clouds with a variable number of attributes at runtime. LidarFormat is
distributed under the CeCILL-B licence. See Licence_CeCILL-B_V1-en.txt
or http://www.cecill.info for more details.


Homepage:

	https://fullanalyze.ign.fr/trac/LidarFormat

Copyright:

	Institut Geographique National & CEMAGREF (2009)

Author:

	Adrien Chauve



This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use,
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info".

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability.

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or
data to be ensured and,  more generally, to use and operate it in the
same conditions as regards security.

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms.

***********************************************************************/

/*
 * LidarDataView.hpp
 *
 *  Created on: 28 juil. 2009
 *      Author: ndavid
 */

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
