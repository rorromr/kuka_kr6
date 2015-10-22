const KUKA_VARNAME_LEN = 14;

const KUKA_STRING_LEN = 128; 

typedef float kukaReal_t;

typedef int kukaInt_t;

typedef int kukaBool_t;

typedef char kukaChar_t;

typedef kukaChar_t kukaString_t[KUKA_STRING_LEN];

struct kukaAxis_s
{
 kukaReal_t a1;
 kukaReal_t a2;
 kukaReal_t a3;
 kukaReal_t a4;
 kukaReal_t a5;
 kukaReal_t a6;
};
typedef kukaAxis_s kukaAxis_t;

struct kukaFrame_s
{
 kukaReal_t x;
 kukaReal_t y;
 kukaReal_t z;
 kukaReal_t a;
 kukaReal_t b;
 kukaReal_t c;
};
typedef kukaFrame_s kukaFrame_t;

struct kukaPos_s
{
 kukaReal_t x;
 kukaReal_t y;
 kukaReal_t z;
 kukaReal_t a;
 kukaReal_t b;
 kukaReal_t c;
 kukaInt_t s;
 kukaInt_t t;
};
typedef kukaPos_s kukaPos_t;

enum kukaErrorType_e {
  KUKA_KUKA_E           = 0x001a,
  KUKA_CROSS_E          = 0x002a,
  KUKA_RPC_E            = 0x003a,
  KUKA_API_E            = 0x004a
};
typedef kukaErrorType_e kukaErrorType_t;
  
struct kukaError_s
{
  kukaErrorType_t type;
  long no;
  kukaString_t desc;
};
typedef kukaError_s kukaError_t;

enum kukaType_e{
  /*---------------------------*/
  KUKA_UNKNOWN          = 0x0000,
  /*---------------------------*/
  KUKA_SIMPLE           = 0x0001,
  KUKA_INT              = 0x0011,
  KUKA_REAL             = 0x0021,
  KUKA_BOOL             = 0x0031,
  KUKA_CHAR             = 0x0041,
  KUKA_STRING           = 0x0051,
  /*---------------------------*/
  KUKA_STRUCT           = 0x0002,
  KUKA_AXIS             = 0x0012,
  KUKA_FRAME            = 0x0022,
  KUKA_POS              = 0x0032,
  /*---------------------------*/
  KUKA_ERROR            = 0x000a
};
typedef kukaType_e kukaType_t;

union kukaVal switch (kukaType_t type) {
 case KUKA_INT:
   kukaInt_t kukaInt;
 case KUKA_REAL:
   kukaReal_t kukaReal;
 case KUKA_BOOL:
   kukaBool_t kukaBool;
 case KUKA_CHAR:
   kukaChar_t kukaChar;
 case KUKA_STRING:
   kukaString_t kukaString;

 case KUKA_AXIS:
   kukaAxis_t kukaAxis;
 case KUKA_FRAME:
   kukaFrame_t kukaFrame;
 case KUKA_POS:
   kukaPos_t kukaPos;

 case KUKA_ERROR :
   kukaError_t kukaError;

 default:
   void;
};

struct kukaVar_s
{
  char nom[KUKA_VARNAME_LEN];
  kukaVal valeur;
};
typedef kukaVar_s kukaVar_t;

program KUKAPROG {
  version KUKAVERS {
    kukaVar_t INITIALIZE(void) = 1;
    kukaVar_t UNINITIALIZE(void) = 2;

    kukaVar_t GETVAR(kukaVar_t) = 11;
    kukaVar_t SETVAR(kukaVar_t) = 12;

    kukaVar_t LOADMODULE(string) = 21;
    kukaVar_t UNLOADMODULE(void) = 22;

    kukaVar_t GETCROSSERROR(void) = 31;
  } = 1;
} = 0x20000002 ;




