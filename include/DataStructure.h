#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

class DataStructure
{
public:
	// DataStructure() {}

	virtual void display() const = 0;

	virtual bool isEmpty() const = 0;

	virtual void clear() = 0;

	virtual ~DataStructure() {}
};

#endif // DATASTRUCTURE_H