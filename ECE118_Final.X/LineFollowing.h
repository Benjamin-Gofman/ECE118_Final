#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

// Sets tapeFound to 0 or 1
void Set_TapeFound(unsigned char enable);

// Return 1 while tape-following
unsigned char LineFollowing(const unsigned char onTape[6],
        unsigned char drivingInReverse);

#endif