#include <rpc/types.h>
#define KUKA_VARNAME_LEN 14
#define KUKA_STRING_LEN 128

typedef float kukaReal_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaReal_t(...);
}
#else
bool_t xdr_kukaReal_t();
#endif


typedef int kukaInt_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaInt_t(...);
}
#else
bool_t xdr_kukaInt_t();
#endif


typedef int kukaBool_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaBool_t(...);
}
#else
bool_t xdr_kukaBool_t();
#endif


typedef char kukaChar_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaChar_t(...);
}
#else
bool_t xdr_kukaChar_t();
#endif


typedef kukaChar_t kukaString_t[KUKA_STRING_LEN];
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaString_t(...);
}
#else
bool_t xdr_kukaString_t();
#endif


struct kukaAxis_s {
	kukaReal_t a1;
	kukaReal_t a2;
	kukaReal_t a3;
	kukaReal_t a4;
	kukaReal_t a5;
	kukaReal_t a6;
};
typedef struct kukaAxis_s kukaAxis_s;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaAxis_s(...);
}
#else
bool_t xdr_kukaAxis_s();
#endif


typedef kukaAxis_s kukaAxis_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaAxis_t(...);
}
#else
bool_t xdr_kukaAxis_t();
#endif


struct kukaFrame_s {
	kukaReal_t x;
	kukaReal_t y;
	kukaReal_t z;
	kukaReal_t a;
	kukaReal_t b;
	kukaReal_t c;
};
typedef struct kukaFrame_s kukaFrame_s;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaFrame_s(...);
}
#else
bool_t xdr_kukaFrame_s();
#endif


typedef kukaFrame_s kukaFrame_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaFrame_t(...);
}
#else
bool_t xdr_kukaFrame_t();
#endif


struct kukaPos_s {
	kukaReal_t x;
	kukaReal_t y;
	kukaReal_t z;
	kukaReal_t a;
	kukaReal_t b;
	kukaReal_t c;
	kukaInt_t s;
	kukaInt_t t;
};
typedef struct kukaPos_s kukaPos_s;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaPos_s(...);
}
#else
bool_t xdr_kukaPos_s();
#endif


typedef kukaPos_s kukaPos_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaPos_t(...);
}
#else
bool_t xdr_kukaPos_t();
#endif


enum kukaErrorType_e {
	KUKA_KUKA_E = 0x001a,
	KUKA_CROSS_E = 0x002a,
	KUKA_RPC_E = 0x003a,
	KUKA_API_E = 0x004a,
};
typedef enum kukaErrorType_e kukaErrorType_e;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaErrorType_e(...);
}
#else
bool_t xdr_kukaErrorType_e();
#endif


typedef kukaErrorType_e kukaErrorType_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaErrorType_t(...);
}
#else
bool_t xdr_kukaErrorType_t();
#endif


struct kukaError_s {
	kukaErrorType_t type;
	long no;
	kukaString_t desc;
};
typedef struct kukaError_s kukaError_s;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaError_s(...);
}
#else
bool_t xdr_kukaError_s();
#endif


typedef kukaError_s kukaError_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaError_t(...);
}
#else
bool_t xdr_kukaError_t();
#endif


enum kukaType_e {
	KUKA_UNKNOWN = 0x0000,
	KUKA_SIMPLE = 0x0001,
	KUKA_INT = 0x0011,
	KUKA_REAL = 0x0021,
	KUKA_BOOL = 0x0031,
	KUKA_CHAR = 0x0041,
	KUKA_STRING = 0x0051,
	KUKA_STRUCT = 0x0002,
	KUKA_AXIS = 0x0012,
	KUKA_FRAME = 0x0022,
	KUKA_POS = 0x0032,
	KUKA_ERROR = 0x000a,
};
typedef enum kukaType_e kukaType_e;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaType_e(...);
}
#else
bool_t xdr_kukaType_e();
#endif


typedef kukaType_e kukaType_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaType_t(...);
}
#else
bool_t xdr_kukaType_t();
#endif


struct kukaVal {
	kukaType_t type;
	union {
		kukaInt_t kukaInt;
		kukaReal_t kukaReal;
		kukaBool_t kukaBool;
		kukaChar_t kukaChar;
		kukaString_t kukaString;
		kukaAxis_t kukaAxis;
		kukaFrame_t kukaFrame;
		kukaPos_t kukaPos;
		kukaError_t kukaError;
	} kukaVal_u;
};
typedef struct kukaVal kukaVal;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaVal(...);
}
#else
bool_t xdr_kukaVal();
#endif


struct kukaVar_s {
	char nom[KUKA_VARNAME_LEN];
	kukaVal val;
};
typedef struct kukaVar_s kukaVar_s;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaVar_s(...);
}
#else
bool_t xdr_kukaVar_s();
#endif


typedef kukaVar_s kukaVar_t;
#ifdef __cplusplus
extern "C" {
bool_t xdr_kukaVar_t(...);
}
#else
bool_t xdr_kukaVar_t();
#endif


#define KUKAPROG ((u_long)0x20000002)
#define KUKAVERS ((u_long)1)
#define INITIALIZE ((u_long)1)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *initialize_1(...);
}
#else
extern kukaVar_t *initialize_1();
#endif /* __cplusplus */
#define UNINITIALIZE ((u_long)2)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *uninitialize_1(...);
}
#else
extern kukaVar_t *uninitialize_1();
#endif /* __cplusplus */
#define GETVAR ((u_long)11)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *getvar_1(...);
}
#else
extern kukaVar_t *getvar_1();
#endif /* __cplusplus */
#define SETVAR ((u_long)12)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *setvar_1(...);
}
#else
extern kukaVar_t *setvar_1();
#endif /* __cplusplus */
#define LOADMODULE ((u_long)21)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *loadmodule_1(...);
}
#else
extern kukaVar_t *loadmodule_1();
#endif /* __cplusplus */
#define UNLOADMODULE ((u_long)22)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *unloadmodule_1(...);
}
#else
extern kukaVar_t *unloadmodule_1();
#endif /* __cplusplus */
#define GETCROSSERROR ((u_long)31)
#ifdef __cplusplus
extern "C" {
extern kukaVar_t *getcrosserror_1(...);
}
#else
extern kukaVar_t *getcrosserror_1();
#endif /* __cplusplus */

