#include "define.h"

class BodyDataLoader
{
public:
	BodyDataLoader(void);
	~BodyDataLoader(void);

	/*mode 'w' : write mode
	mode 'r' : read mode*/
	void OpenDataFile(char *filename, char mod);
	void CloseDataFile();

	void ReadData(BodyJoint *dst);
	void WriteData(BodyJoint *src);

	void ReadAllData();
	void GetBodyData(BodyJoint *dst);

private:
	FILE *Datafp;

	std::list<BodyJoint> BodyDatalist;
};

