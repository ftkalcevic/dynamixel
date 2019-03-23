// test12bit.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>

static void test(uint16_t old_pos, uint16_t pos, int16_t expexted_delta)
{
	int16_t delta = pos - old_pos;

	if (delta & 0x0800)
		delta = delta | 0xF000;
	else
		delta = delta & 0x0FFF;

	std::cout << old_pos << ", ";
	std::cout << pos << ", ";
	std::cout << delta << " (" << expexted_delta << ")\n";
}


int main()
{
	test(1, 2, 1);
	test(2, 1, -1);
	test(120,1, -119);
	test(4095, 0, +1);
	test(0, 4095, -1);

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
