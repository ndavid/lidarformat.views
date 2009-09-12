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


#include <iostream>
#include <boost/filesystem.hpp>

#include <boost/progress.hpp>

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
	std::cout<<std::endl<<" debut index view"<<std::endl;
	std::cout<<" \t index 2 4 6 8 "<<std::endl;
	boost::shared_ptr<std::vector<int> > index_ptr(new std::vector<int> );
	index_ptr->push_back(2);
	index_ptr->push_back(4);
	index_ptr->push_back(6);
	index_ptr->push_back(8);
	typedef LidarDataAttProxyIndexView<double,3> index_view_type;
	typedef index_view_type::iterator xyz_index_ite_type;
	typedef xyz_index_ite_type::reference xzy_index_echo_type;
	index_view_type xzy_indexview=index_view_type(lidarcontainer_ptr,index_ptr, echo_stride, x_offset,z_offset,y_offset);
	xyz_index_ite_type xzy_index_ite;
	xyz_index_ite_type xzy_index_begin=xzy_indexview.begin();
	std::cout<<" begin "<<(*xzy_index_begin).get<0>()<<" "<<(*xzy_index_begin).get<1>()<<" "<<(*xzy_index_begin).get<2>()<<std::endl;
	xyz_index_ite_type xzy_index_end=xzy_indexview.end();
	--xzy_index_end;
	std::cout<<" end -1 "<<(*xzy_index_end).get<0>()<<" "<<(*xzy_index_end).get<1>()<<" "<<(*xzy_index_end).get<2>()<<std::endl;
	for(xzy_index_ite=xzy_indexview.begin(); xzy_index_ite!=xzy_indexview.end(); xzy_index_ite++)
	{
		std::cout<<(*xzy_index_ite).get<0>()<<" "<<(*xzy_index_ite).get<1>()<<" "<<(*xzy_index_ite).get<2>()<<std::endl;
	}
	std::cout<<" fin view"<<std::endl;

	return 0;
}


void checkPerformance()
{
	boost::shared_ptr<LidarDataContainer> lidarContainer(new LidarDataContainer() );

	lidarContainer->addAttribute("x", LidarDataType::float64);
//	lidarContainer->addAttribute("y", LidarDataType::float64);
//	lidarContainer->addAttribute("z", LidarDataType::float64);
	const std::size_t taille = 50000000;
	lidarContainer->resize(taille);



	{
		std::cout << "\n\nIterator Attribute:\n";


		const LidarConstIteratorAttribute<double> ite = lidarContainer->endAttribute<double>("x");
		const LidarConstIteratorAttribute<double> itbegin = lidarContainer->beginAttribute<double>("x");

		boost::progress_timer t; //start timing
		for(unsigned int k=0; k < 50; ++k)
		{
			LidarConstIteratorAttribute<double> itb = itbegin;
			for( ; itb != ite; ++itb)
				*itb = 1.0;
		}

	}


	{
		std::cout << "\n\nAttViewIterator:\n";

		const unsigned int x_offset = lidarContainer->getDecalage("x");
		const unsigned int echo_stride = lidarContainer->pointSize();
		LidarDataAttView<double> x_view = LidarDataAttView<double>(lidarContainer, x_offset, echo_stride);
		typedef LidarDataAttView<double>::iterator x_ite_type;


		const x_ite_type ite = x_view.end();
		const x_ite_type itbegin = x_view.begin();

		boost::progress_timer t; //start timing
		for(unsigned int k=0; k < 50; ++k)
		{
			x_ite_type itb = itbegin;
			for( ; itb != ite; ++itb)
				*itb = 1.0;
		}

	}


	{
		std::cout << "\n\nvector:\n";

		typedef std::vector<double> ContainerType;
		ContainerType container(taille);


		const ContainerType::iterator ite = container.end();
		const ContainerType::iterator itbegin = container.begin();

		boost::progress_timer t; //start timing
		for(unsigned int k=0; k < 50; ++k)
		{
			ContainerType::iterator itb = itbegin;
			for( ; itb != ite; ++itb)
				*itb = 1.0;
		}

	}

}

