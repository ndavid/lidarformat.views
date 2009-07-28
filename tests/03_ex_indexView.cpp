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
	LidarDataContainer lidarContainer;

	//Load data from the file to the container
	file.loadData(lidarContainer);



	/**** Print data in the console ****/

	std::cout << "\n\nContainer content:\n";

	//Print the list of attributes
	lidarContainer.printHeader(cout);

	//Print the content of the container
	ostream_iterator<LidarEcho> echoOutputIterator( cout, "\n" );
	copy(lidarContainer.begin(), lidarContainer.end(), echoOutputIterator);





	return 0;
}

