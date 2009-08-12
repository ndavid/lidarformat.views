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

	Nicolas DAVID



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
 * 03_ex_indexView.cpp
 *
 *  Created on: 28 juil. 2009
 *      Author: ndavid
 */
#include <iostream>
#include <boost/filesystem.hpp>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/LidarDataViewElement.hpp"
#include "LidarFormat/LidarDataView.hpp"


int main()
{
	using namespace Lidar;
	using namespace std;



	/**** Load data in a container ****/

	//data test filename (trunk/data/testAscii.xml)
	const string lidarFileName(string(PATH_LIDAR_TEST_DATA) + "/indexview/AllPoint.xml");

	//create a LidarFile which reads the XML file and contains informations about the data
	//(size, number of attributes, types of attributes, path to the data file)
	LidarFile file(lidarFileName);

	//create an empty LidarDataContainer, the object to contain point clouds
	boost::shared_ptr<LidarDataContainer> lidarcontainer_ptr(new LidarDataContainer() );
	//LidarDataContainer lidarContainer;

	//Load data from the file to the container
	file.loadData(*lidarcontainer_ptr);



	/**** Print data in the console ****/

	std::cout << "\n\nContainer content:\n";

	//Print the list of attributes
	lidarcontainer_ptr->printHeader(cout);

	//Print the content of the container
	ostream_iterator<LidarEcho> echoOutputIterator( cout, "\n" );
	copy(lidarcontainer_ptr->begin(), lidarcontainer_ptr->end(), echoOutputIterator);

	//****************************************************************************************
	//***************************************************************************************
	// testing iterations on Simple Attribute
	//****************************************************************************************
	//****************************************************************************************
	//test only Att iterator
	std::cout << "\n\n x iterator test \n";
	AttViewIterator<double> x_iterator=AttViewIterator<double>(lidarcontainer_ptr->rawData(), 24);
	for (int i=0; i<10 ; i++)
	{
		std::cout<<*x_iterator<<std::endl;
		x_iterator++;
	}
	// test AttView
	std::cout << "\n\n x view test \n";
	unsigned int x_offset=lidarcontainer_ptr->getDecalage("x");
	unsigned int echo_stride=lidarcontainer_ptr->pointSize();
	LidarDataAttView<double> x_view=LidarDataAttView<double>(lidarcontainer_ptr, x_offset,echo_stride);
	typedef LidarDataAttView<double>::iterator x_ite_type;
	x_ite_type x_ite;
	x_ite_type x_begin=x_view.begin();
	x_ite_type x_end=x_view.end();
	--x_end;
	std::cout << " x begin "<<*x_begin<<std::endl;
	std::cout<<" x end "<<*(x_end)<<std::endl;

	for(x_ite=x_view.begin(); x_ite!=x_view.end(); x_ite++)
	{
		std::cout<<*x_ite<<std::endl;
	}
	std::cout<<" fin view"<<std::endl;
		//Print the content of the container
	//copy(x_view.begin(), x_view.end(), ostream_iterator<double>(cout, "\n"));
	 // pb need copy algorithm seems to need const ite in input

	//****************************************************************************************
	//****************************************************************************************
	// testing iterations on multiple value iterator
	//****************************************************************************************
	//****************************************************************************************
	//EnumLidarDataType x_type=lidarContainer.getAttributeType("x");
	std::cout << "\n\n xz iterator test \n";
	AttViewProxyIterator<double, 2> xz_iterator=AttViewProxyIterator<double,2>(lidarcontainer_ptr->rawData(),echo_stride, 0,16);
	typedef AttViewProxyIterator<double, 2>::reference xz_echo_type;
		for (int i=0; i<10 ; i++)
		{
			xz_echo_type xz_echo;
			xz_echo=*xz_iterator;
			std::cout<<xz_echo.get<0>()<<" "<<xz_echo.get<1>()<<std::endl;
			xz_iterator++;
		}

	// test AttView
	std::cout << "\n\n xzy view test \n";
	unsigned int z_offset=lidarcontainer_ptr->getDecalage("z");
	unsigned int y_offset=lidarcontainer_ptr->getDecalage("y");
	LidarDataAttProxyView<double,3> xzy_view=LidarDataAttProxyView<double,3>(lidarcontainer_ptr, echo_stride, x_offset,z_offset,y_offset);
	typedef LidarDataAttProxyView<double,3>::iterator xzy_ite_type;
	typedef xzy_ite_type::reference xzy_echo_type;

	xzy_ite_type xzy_ite;
	xzy_ite_type xzy_begin=xzy_view.begin();
	std::cout<<" begin "<<(*xzy_begin).get<0>()<<" "<<(*xzy_begin).get<1>()<<" "<<(*xzy_begin).get<2>()<<std::endl;
	xzy_ite_type xzy_end=xzy_view.end();
	--xzy_end;
	std::cout<<" end -1 "<<(*xzy_end).get<0>()<<" "<<(*xzy_end).get<1>()<<" "<<(*xzy_end).get<2>()<<std::endl;
	if(xzy_begin==xzy_end){std::cout<<" Pb begin and end iterator are same for equal comparison"<<std::endl;}
	for(xzy_ite=xzy_view.begin(); xzy_ite!=xzy_view.end(); xzy_ite++)
	{
		std::cout<<(*xzy_ite).get<0>()<<" "<<(*xzy_ite).get<1>()<<" "<<(*xzy_ite).get<2>()<<std::endl;
	}
	std::cout<<" fin view"<<std::endl;

	//****************************************************************************************
	//****************************************************************************************
	// indexing iterations
	//****************************************************************************************
	//****************************************************************************************

	return 0;
}

