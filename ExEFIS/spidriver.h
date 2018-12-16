#pragma once
class spidriver
{
public:
	spidriver(const char* device);
	~spidriver();
	void init(void);
	int transferdata(unsigned char* data, int length);
	
protected:
	int fd;
	
};

