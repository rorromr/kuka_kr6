#include <rpc/rpc.h>
#include "kuka.h"

/* Default timeout can be changed using clnt_control() */
static struct timeval TIMEOUT = { 25, 0 };

kukaVar_t *
initialize_1(argp, clnt)
	void *argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, INITIALIZE, xdr_void, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}


kukaVar_t *
uninitialize_1(argp, clnt)
	void *argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, UNINITIALIZE, xdr_void, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}


kukaVar_t *
getvar_1(argp, clnt)
	kukaVar_t *argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, GETVAR, xdr_kukaVar_t, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}


kukaVar_t *
setvar_1(argp, clnt)
	kukaVar_t *argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, SETVAR, xdr_kukaVar_t, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}


kukaVar_t *
loadmodule_1(argp, clnt)
	char **argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, LOADMODULE, xdr_wrapstring, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}


kukaVar_t *
unloadmodule_1(argp, clnt)
	void *argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, UNLOADMODULE, xdr_void, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}


kukaVar_t *
getcrosserror_1(argp, clnt)
	void *argp;
	CLIENT *clnt;
{
	static kukaVar_t res;

	bzero((char *)&res, sizeof(res));
	if (clnt_call(clnt, GETCROSSERROR, xdr_void, argp, xdr_kukaVar_t, &res, TIMEOUT) != RPC_SUCCESS) {
		return (NULL);
	}
	return (&res);
}

