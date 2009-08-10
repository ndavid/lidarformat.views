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


//************************************************************
//***********************************************************
// simple value type proxy elements templated on dimension
//*********************************************************
 //*******************************************************
template<typename AttType1, int dim>
class ViewProxyElement{
 public :
	ViewProxyElement(){}

};

template<typename AttType1, int dim>
class ViewProxyElement<AttType1 , 1>{

	public :
		ViewProxyElement():value0(0){}
		ViewProxyElement(AttType1* val0)
			:value0(val0) {}
		template<int dim> AttType1 get();
		template<int dim> AttType1 get<0>() {return *value0;}

		template<int dim> void set(AttType1 value);
		template<int dim> void set<0>(AttType1 value) { *value0=value;}

	private :
	AttType1* value0;
};

template<typename AttType1, int dim>
class ViewProxyElement<AttType1 , 2>{

	public :
		ViewProxyElement():value0(0),value1(0) {}
		ViewProxyElement(AttType1* val0,AttType1* val1)
			:value0(val0),
			value1(val1) {}
		template<int dim> AttType1 get();
		template<int dim> AttType1 get<0>() {return *value0;}
		template<int dim> AttType1 get<1>() {return *value1;}

		template<int dim> void set(AttType1 value);
		template<int dim> void set<0>(AttType1 value) { *value0=value;}
		template<int dim> void set<1>(AttType1 value) { *value1=value;}

	private :
	AttType1* value0;
	AttType1* value1;
};

template<typename AttType1, int dim>
class ViewProxyElement<AttType1, 3>{

	public :
		ViewProxyElement():value0(0), value1(0), value2(0){}
		ViewProxyElement(AttType1* val0,AttType1* val1,AttType1* val2)
			:value0(val0),
			value1(val1),
			value2(val2){}
		ViewProxyElement()
			:value0(0),
			value1(0),
			value2(0){}
		template<int dim> AttType1 get();
		template<int dim> AttType1 get<0>() {return *value0;}
		template<int dim> AttType1 get<1>() {return *value1;}
		template<int dim> AttType1 get<2>() {return *value2;}

		template<int dim> void set(AttType1 value);
		template<int dim> void set<0>(AttType1 value) { *value0=value;}
		template<int dim> void set<1>( AttType1 value) { *value1=value;}
		template<int dim> void set<2>( AttType1 value) { *value2=value;}

	private :
	AttType1* value0;
	AttType1* value1;
	AttType1* value2;
};

template<typename AttType, int dim>
ViewProxyElement<AttType, dim> MakeViewProxyElement(char*, unsigned int *)
{ }

inline
template<typename AttType, int dim>
ViewProxyElement<AttType, 1> MakeViewProxyElement<AttType, 1>(char* raw_data, unsigned int * offsets)
{
 return ViewProxyElement<AttType, 1>(*reinterpret_cast<AttType*>(raw_data+*offset)) ;
}

inline
template<typename AttType, int dim>
ViewProxyElement<AttType, 2> MakeViewProxyElement<AttType, 2>(char* raw_data, unsigned int * offsets)
{
 return ViewProxyElement<AttType, 2>(*reinterpret_cast<AttType*>(raw_data+*offset),
		 *reinterpret_cast<AttType*>(raw_data+*(offset+1))) ;
}

inline
template<typename AttType, int dim>
ViewProxyElement<AttType, 3> MakeViewProxyElement<AttType, 3>(char* raw_data, unsigned int * offsets)
{
 return ViewProxyElement<AttType, 3>(*reinterpret_cast<AttType*>(raw_data+*offset),
		 *reinterpret_cast<AttType*>(raw_data+*(offset+1)),
		 *reinterpret_cast<AttType*>(raw_data+*(offset+2)));
}

//*******************************************************
// iterator templated on dim, only with proxy element
template <typename AttType, int dim>
class AttViewProxyIterator
  : public boost::iterator_facade<
      AttViewProxyIterator<AttType, dim>
      , ViewProxyElement<AttType, dim>
      , boost::random_access_traversal_tag
      , ViewProxyElement<AttType, dim>&
      ,std::ptrdiff_t
    >
{
public :
		AttViewProxyIterator()
	      : m_raw_data(0), m_stride(0)
	       {
	    	for(int i=0; i<dim ;i++){	m_offsets[i]=0;}
	       }

	    explicit AttViewProxyIterator(char* raw_data, unsigned int stride,
	    		unsigned int offset0=0,
	    		unsigned int offset1=0,
	    		unsigned int offset2=0,
	    		unsigned int offset3=0,
	    		)
	      : m_raw_data(raw_data),m_stride(stride)
		{
	    	  if (dim >= 1) m_offsets[0] = offset0;
	    	  if (dim >= 2) m_offsets[1] = offset1;
	    	   if (dim >= 3) m_offsets[2] = offset2;
	    	   if (dim >= 3) m_offsets[3] = offset3;
		}

	 private:
	    friend class boost::iterator_core_access;

		//*****************************************************
		//implement the iterator random access façade interface
	    void increment() { m_raw_data += m_stride; }

	    bool equal(AttViewIterator const& other) const
	    {
	        return ( (this->m_raw_data == other.m_raw_data) && (m_stride==other.m_stride));
	    }

	    ViewProxyElement<AttType, dim>& dereference() const {

	    	return MakeViewProxyElement<AttType, dim> (m_raw_data, &m_offsets);

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
    unsigned int m_offsets[dim];
    //ViewProxyElement<AttType, dim> m_proxy_element;
};

#endif /* LIDARDATAVIEWELEMENT_HPP_ */
