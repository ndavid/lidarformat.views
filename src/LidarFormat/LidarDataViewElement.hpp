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
 * LidarDataViewElement.hpp
 *
 *  Created on: 28 juil. 2009
 *      Author: ndavid
 */

#ifndef LIDARDATAVIEWELEMENT_HPP_
#define LIDARDATAVIEWELEMENT_HPP_

#include <boost/iterator/iterator_facade.hpp>

template <typename AttType>
class AttViewIterator
  : public boost::iterator_facade<
      AttViewIterator<AttType>
      , AttType
      , boost::random_access_traversal_tag
      , AttType&
      ,std::ptrdiff_t
    >
{
public:
    AttViewIterator()
      : m_raw_data(0),
       m_stride(0) {}

    explicit AttViewIterator(char* raw_data, unsigned int stride)
      : m_raw_data(raw_data), m_stride(stride) {}

 private:
    friend class boost::iterator_core_access;

	//*****************************************************
	//implement the iterator random access façade interface
    void increment() { m_raw_data += m_stride; }

    bool equal(AttViewIterator const& other) const
    {
        return this->m_raw_data == other.m_raw_data;
    }

    AttType& dereference() const { 
    	return *reinterpret_cast<AttType*>(m_raw_data);
    }
    
    void decrement() { m_raw_data -=m_stride;}
    
    void advance(int n) {m_raw_data += n*m_stride; }

 	std::ptrdiff_t distance_to(AttViewIterator j) 
 	{
 		assert(m_stride == j.m_stride);
		return static_cast<std::ptrdiff_t >( (m_raw_data - j.m_raw_data) / m_stride);
 	
	}	
	
	//*******************************************************
	// private meber data
	// store a raw_data pointer of char*
    char* m_raw_data;
    unsigned int m_stride;
};



//***********************************************************
// proxy elements templated on dimension
template<typename AttType1, int dim>
class ViewProxyElement{
 public :
	ViewProxyElement(){}
	
}

template<typename AttType1, 0>
class ViewProxyElement{
	public :
	ViewProxyElement(){}
}

template<typename AttType1, 1>
class ViewProxyElement{
	
	public :
		ViewProxyElement(AttType1* val0)
			:value0(val0) {}
		template<int dim> AttType1 get();
		template<0> AttType1 get() {return *value0;}
		
		template<int dim> AttType1 set();
		template<0> set( AttType1 value) { *value0=value;}
		
	private :	
	AttType1* value0;
}

template<typename AttType1, 2>
class ViewProxyElement{
	
	public :
		ViewProxyElement(AttType1* val0,AttType1* val1)
			:value0(val0),
			value1(val1) {}
		template<int dim> AttType1 get();
		template<0> AttType1 get() {return *value0;}
		template<1> AttType1 get() {return *value1;}
		
		template<int dim> AttType1 set();
		template<0> set( AttType1 value) { *value0=value;}
		template<1> set( AttType1 value) { *value1=value;}
		
	private :	
	AttType1* value0;
	AttType1* value1;
}

template<typename AttType1, 3>
class ViewProxyElement{
	
	public :
		ViewProxyElement(AttType1* val0,AttType1* val1,AttType1* val2)
			:value0(val0),
			value1(val1),
			value2(val2){}
		template<int dim> AttType1 get();
		template<0> AttType1 get() {return *value0;}
		template<1> AttType1 get() {return *value1;}
		template<2> AttType1 get() {return *value2;}
		
		template<int dim> AttType1 set();
		template<0> set( AttType1 value) { *value0=value;}
		template<1> set( AttType1 value) { *value1=value;}
		template<2> set( AttType1 value) { *value2=value;}
		
	private :	
	AttType1* value0;
	AttType1* value1;
	AttType1* value2;
}

//*******************************************************
// iterator templated on dim, only with proxy element
template <typename AttType, int dim>
class AttViewProxyIterator
  : public boost::iterator_facade<
      AttViewProxyIterator<AttType,int dim>
      , AttType
      , boost::random_access_traversal_tag
      , ViewProxyElement<AttType, dim>&
      ,std::ptrdiff_t
    >
{

};

#endif /* LIDARDATAVIEWELEMENT_HPP_ */
