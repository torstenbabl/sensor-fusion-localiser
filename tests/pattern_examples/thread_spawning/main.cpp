/// @brief Simple program to test the build system
/// @author Torsten Babl

#include <iostream>
#include <thread>

using std::thread, std::this_thread::sleep_for, std::cout, std::endl;
using namespace std::chrono_literals;

void functionOne()
{
  sleep_for(10ms);
  cout << "threadOne: Message" << endl;

}

void functionTwo()
{
  sleep_for(20ms);
  cout << "threadTwo: Message" << endl;
}

int main()
{
  cout << "main: Enter main loop" << endl;

  thread threadOne(functionOne);
  thread threadTwo(functionTwo);

  cout << "main: Waiting for threads to join" << endl;
  threadOne.join();
  threadTwo.join();
  cout << "main: All done, exiting" << endl;

  return 0;
}