/*
 * Please do not edit this file.
 * It was generated using rpcgen.
 */

#include "kuka.h"

bool_t
xdr_kukaReal_t (XDR *xdrs, kukaReal_t *objp)
{
	register int32_t *buf;

	 if (!xdr_float (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaInt_t (XDR *xdrs, kukaInt_t *objp)
{
	register int32_t *buf;

	 if (!xdr_int (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaBool_t (XDR *xdrs, kukaBool_t *objp)
{
	register int32_t *buf;

	 if (!xdr_int (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaChar_t (XDR *xdrs, kukaChar_t *objp)
{
	register int32_t *buf;

	 if (!xdr_char (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaString_t (XDR *xdrs, kukaString_t objp)
{
	register int32_t *buf;

	 if (!xdr_vector (xdrs, (char *)objp, KUKA_STRING_LEN,
		sizeof (kukaChar_t), (xdrproc_t) xdr_kukaChar_t))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaAxis_s (XDR *xdrs, kukaAxis_s *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaReal_t (xdrs, &objp->a1))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a2))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a3))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a4))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a5))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a6))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaAxis_t (XDR *xdrs, kukaAxis_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaAxis_s (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaFrame_s (XDR *xdrs, kukaFrame_s *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaReal_t (xdrs, &objp->x))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->y))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->z))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->b))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->c))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaFrame_t (XDR *xdrs, kukaFrame_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaFrame_s (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaPos_s (XDR *xdrs, kukaPos_s *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaReal_t (xdrs, &objp->x))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->y))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->z))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->a))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->b))
		 return FALSE;
	 if (!xdr_kukaReal_t (xdrs, &objp->c))
		 return FALSE;
	 if (!xdr_kukaInt_t (xdrs, &objp->s))
		 return FALSE;
	 if (!xdr_kukaInt_t (xdrs, &objp->t))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaPos_t (XDR *xdrs, kukaPos_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaPos_s (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaErrorType_e (XDR *xdrs, kukaErrorType_e *objp)
{
	register int32_t *buf;

	 if (!xdr_enum (xdrs, (enum_t *) objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaErrorType_t (XDR *xdrs, kukaErrorType_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaErrorType_e (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaError_s (XDR *xdrs, kukaError_s *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaErrorType_t (xdrs, &objp->type))
		 return FALSE;
	 if (!xdr_long (xdrs, &objp->no))
		 return FALSE;
	 if (!xdr_kukaString_t (xdrs, objp->desc))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaError_t (XDR *xdrs, kukaError_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaError_s (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaType_e (XDR *xdrs, kukaType_e *objp)
{
	register int32_t *buf;

	 if (!xdr_enum (xdrs, (enum_t *) objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaType_t (XDR *xdrs, kukaType_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaType_e (xdrs, objp))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaVal (XDR *xdrs, kukaVal *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaType_t (xdrs, &objp->type))
		 return FALSE;
	switch (objp->type) {
	case KUKA_INT:
		 if (!xdr_kukaInt_t (xdrs, &objp->kukaVal_u.kukaInt))
			 return FALSE;
		break;
	case KUKA_REAL:
		 if (!xdr_kukaReal_t (xdrs, &objp->kukaVal_u.kukaReal))
			 return FALSE;
		break;
	case KUKA_BOOL:
		 if (!xdr_kukaBool_t (xdrs, &objp->kukaVal_u.kukaBool))
			 return FALSE;
		break;
	case KUKA_CHAR:
		 if (!xdr_kukaChar_t (xdrs, &objp->kukaVal_u.kukaChar))
			 return FALSE;
		break;
	case KUKA_STRING:
		 if (!xdr_kukaString_t (xdrs, objp->kukaVal_u.kukaString))
			 return FALSE;
		break;
	case KUKA_AXIS:
		 if (!xdr_kukaAxis_t (xdrs, &objp->kukaVal_u.kukaAxis))
			 return FALSE;
		break;
	case KUKA_FRAME:
		 if (!xdr_kukaFrame_t (xdrs, &objp->kukaVal_u.kukaFrame))
			 return FALSE;
		break;
	case KUKA_POS:
		 if (!xdr_kukaPos_t (xdrs, &objp->kukaVal_u.kukaPos))
			 return FALSE;
		break;
	case KUKA_ERROR:
		 if (!xdr_kukaError_t (xdrs, &objp->kukaVal_u.kukaError))
			 return FALSE;
		break;
	default:
		break;
	}
	return TRUE;
}

bool_t
xdr_kukaVar_s (XDR *xdrs, kukaVar_s *objp)
{
	register int32_t *buf;

	int i;
	 if (!xdr_vector (xdrs, (char *)objp->nom, KUKA_VARNAME_LEN,
		sizeof (char), (xdrproc_t) xdr_char))
		 return FALSE;
	 if (!xdr_kukaVal (xdrs, &objp->val))
		 return FALSE;
	return TRUE;
}

bool_t
xdr_kukaVar_t (XDR *xdrs, kukaVar_t *objp)
{
	register int32_t *buf;

	 if (!xdr_kukaVar_s (xdrs, objp))
		 return FALSE;
	return TRUE;
}
