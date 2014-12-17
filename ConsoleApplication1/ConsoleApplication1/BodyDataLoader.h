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

	void ReadData();
	void WriteData();

private:

	FILE *Datafp;
};

