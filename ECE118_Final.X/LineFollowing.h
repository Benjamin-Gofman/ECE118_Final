#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

// Set tapeTurn to 0
void LineFollowing_Reset(void);

// Return 1 while tape-following
unsigned char LineFollowing(const unsigned char onTape[6],
        unsigned char drivingInReverse);

#endif
