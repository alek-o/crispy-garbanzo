#ifndef GENERATORS_H
#define GENERATORS_H

template<typename T>
class Generators
{
public:
	virtual T generate() = 0;
};

#endif // GENERATORS_H