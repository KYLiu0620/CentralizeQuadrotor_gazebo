#include "TimeDelayBuffer.h"

DelayedData::DelayedData()
{
	data = arma::mat(1,1,arma::fill::zeros);
	number = 0;
	delay = 0;
}

DelayedData::DelayedData(arma::mat input_data, int data_no, int time)
{
	data = input_data;
	number = data_no;
	delay = time;
}

DelayedData::~DelayedData()
{
	//do nothing
}

DelayedBuffer::DelayedBuffer()
{
	lastOutput = DelayedData(arma::mat(1, 1, arma::fill::zeros), 0, 0);
}

DelayedBuffer::DelayedBuffer(int row, int col)
{
	lastOutput = DelayedData(arma::mat(row, col, arma::fill::zeros), 0, 0);
}

DelayedBuffer::~DelayedBuffer()
{
	//do nothing
}

void DelayedBuffer::addData(DelayedData newData)
{
	bool addSuccess = false;

	if (buf.empty())
	{
		buf.push_back(newData);
	}
	else
	{
		for (std::list<DelayedData>::iterator itr = buf.begin(); itr != buf.end(); ++itr)
		{
			if (newData.delay < itr->delay)
			{
				buf.insert(itr, newData);
				addSuccess = true;
				break;
			}
		}

		if (!addSuccess)
			buf.push_back(newData);

	}
	
}

arma::mat DelayedBuffer::getData(int currentTime)
{
	DelayedData temp = lastOutput;
	bool noDataArrived = true;
	std::list<DelayedData>::iterator itr = buf.begin();


	if (!buf.empty())
	{
		while ( itr != buf.end())
		{
			if (currentTime < itr->delay)
			{
				if (noDataArrived)
					return lastOutput.data; 
				break;
			}
			
			else
			{
				noDataArrived = false;
				if (itr->number > temp.number)	//multiple data arrived. choose latest one
				{
					temp = *itr;
				}
				itr = buf.erase(itr);	//erase and shift itr to next one
			}
		}
	}
	else
	{
		return lastOutput.data;
	}
	
	lastOutput = temp;
	return temp.data;
}	//getData
