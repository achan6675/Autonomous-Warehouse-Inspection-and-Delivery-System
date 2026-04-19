// Navigator.hpp
#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <iostream>
struct CommandVelocity {
    double linear;
    double angular;
};

//just stub function -> replace with maryannes code
class InspectionNavigator {
  public:
      CommandVelocity move()  {

        cmd.linear = 0.1;
        cmd.angular = 0.3;
        
        std::cout << "Inspection navigator guiding robot..." << std::endl;

        return cmd;
      }
  private:
          CommandVelocity cmd;  
};



#endif
