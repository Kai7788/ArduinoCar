//
// Created by kai on 13.03.24.
//

#ifndef ARDUINOCAR_INTERFACES_H
#define ARDUINOCAR_INTERFACES_H

class IDrivable{
public:
    virtual ~IDrivable() = default;
    virtual void drive(String) = 0;

};

#endif //ARDUINOCAR_INTERFACES_H
