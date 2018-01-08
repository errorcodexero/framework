#include "drivebase.h"

size_t Drivebase::pdb_location(Drivebase::Side s, size_t which)
{
  size_t ret = 0xffffffff ;
  
  if (s == Drivebase::Side::Left) {
    switch(which) {
    case 0:
      ret = 0 ;
    case 1:
      ret = 1 ;
      break ;
    }
  }
  else {
    switch(which) {
    case 0:
      ret = 2 ;
    case 1:
      ret = 13 ;
      break ;
    }
  }

  return ret ;
}
