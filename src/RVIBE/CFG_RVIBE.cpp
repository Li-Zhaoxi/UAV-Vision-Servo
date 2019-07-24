#include "CFG_RVIBE.h"
#include <stdlib.h>

using namespace std;
void CFG_RVIBE::Write_RVIBECONFIG(string filename)
{
	string file_path = "Config//" + filename + ".cfg";
	std::ofstream outfile(file_path.c_str());
	
	outfile << "[CONFIG --> RVIBE]" << endl;
	outfile << "NUM_SAMPLES=" << NUM_SAMPLES << endl;
	outfile << "MIN_MATCHES=" << MIN_MATCHES << endl;
	outfile << "RADIUS=" << RADIUS << endl;
	outfile << "SUBSAMPLE_FACTOR=" << SUBSAMPLE_FACTOR << endl;
	outfile << "slipeWindow=" << slipeWindow << endl;
	outfile << "rectWindow=" << rectWindow << endl;
	outfile << "minAreaRatio=" << minAreaRatio << endl;
	outfile << "maxRect=" <<maxRect << endl;

	outfile.close();
}

void CFG_RVIBE::Read_RVIBECONFIG(string filename)
{
	string file_path = "Config//" + filename + ".cfg";
	std::ifstream infile(file_path.c_str());
	if(!infile.is_open())
	{
		std::cout << "RVIBE�����ļ�������" << std::endl;
		exit(1);
	}

	char tmp[1000];
	vector<string> keyValue;
	const int varyNumber = 8;
	int lineNum = 0;
	for (int i = 0; i < varyNumber;)
	{
		infile.getline(tmp, 1000);
		lineNum++;
		std::string line(tmp); line = line + '\0';
		size_t pos = line.find('=');
		if (pos == string::npos)
			continue;
		string tmpKey = line.substr(0, pos);// Get the Varies String before the =
		string value = line.substr(pos + 1);
		keyValue.push_back(value);
		i++;
	}
	if (keyValue.size() != varyNumber)
	{
		cout << "Error Data Variaies" << endl;
		exit(0);
	}

	NUM_SAMPLES = atoi(keyValue[0].c_str());
	MIN_MATCHES = atoi(keyValue[1].c_str());
	RADIUS = atoi(keyValue[2].c_str());
	SUBSAMPLE_FACTOR = atoi(keyValue[3].c_str());

	slipeWindow = atoi(keyValue[4].c_str());
	rectWindow = atoi(keyValue[5].c_str());
	minAreaRatio= (float)atof(keyValue[6].c_str());

	maxRect = atoi(keyValue[7].c_str());
	infile.close();
}
