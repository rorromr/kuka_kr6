/*! \file
 *  \version 0.1.0
 *
 */
#include <stdio.h>
#include <string.h>
#include <rpc/rpc.h>
#include "kuka.h" 

/*! Declaration des variables statiques */
static CLIENT *clnt=NULL;
static char *serveur="kuka";
static kukaVar_t kukaError={"KUKA_ERROR",{KUKA_ERROR,{0}}};

/*! D�claration des fonctions priv�es
 *  Voir � la fin du source pour leur description svp.
 */
static int setKukaErrorFromArg(void *arg);
static int setKukaErrorFromResult(void *result);

/*************************************************/
int kuka_initialize(char* _serveur)
{
  kukaVar_t *result=NULL;
  
  char *nom="test_kuka_api";

#ifdef WIN32
  rpc_nt_init();
#endif

  if( _serveur != NULL )
    serveur=_serveur;
  
  if( clnt != NULL ){
    //printf("Seems to be aleready initialized\n");
    return 2;
  }

  // initialisation du client rpc
  //printf("\nkuka_api::initialize call clnt_create(\"%s\",%d,%d,\"tcp\")\n",serveur, KUKAPROG, KUKAVERS); 
  clnt = clnt_create(serveur, KUKAPROG, KUKAVERS, "tcp");
  if( setKukaErrorFromResult(clnt) ) return 1;

  // initialiastion des couches sous jacentes ( crosscommexe/ole puis kuka ) via rpc
  //printf("kuka_api::initialize call initialize_1(...)\n");
  result=initialize_1(&nom, clnt);
  if( setKukaErrorFromResult(result) ) return 1;

  return 0;
}

/*************************************************/
int kuka_uninitialize(void)
{
  kukaVar_t *result=NULL;

  // d�initialiastion des couches sous jacentes ( crosscommexe/ole puis kuka ) via rpc
  if( clnt == NULL )
    return 0;
  
  result=uninitialize_1(NULL, clnt);
  if( setKukaErrorFromResult(result) ) return 1;

  // d�initialisation du client rpc
  clnt_destroy(clnt);

  clnt=NULL; // Peut etre pas necessaire.

#ifdef WIN32
  rpc_nt_exit();
#endif

  return 0;
}

/*************************************************/
int kuka_getVar(kukaVar_t *varInOut)
{
  kukaVar_t *result=NULL;

  if( setKukaErrorFromArg(varInOut) ) return 1;

  result=getvar_1(varInOut, clnt);
  if( setKukaErrorFromResult(result) ) return 1;

  // appel rpc r�ussi : on copie le r�sultat dans varInOut
  memcpy(varInOut,result,sizeof(kukaVar_t));
  
  return 0;
}

/*************************************************/
int kuka_setVar(kukaVar_t *varIn)
{
  kukaVar_t *result=NULL;

  if( setKukaErrorFromArg(varIn) ) return 1;

  result = setvar_1(varIn, clnt);
  if( setKukaErrorFromResult(result) ) return 1;

  return 0;
}

/*************************************************/
int kuka_loadModule(char* module)
{
  kukaVar_t *result=NULL;

  if( setKukaErrorFromArg(module) ) return 1;

  result = loadmodule_1(&module, clnt);
  if( setKukaErrorFromResult(result) ) return 1;

  return 0;
}

/*************************************************/
int kuka_unloadModule(void)
{
  kukaVar_t *result=NULL;

  if( setKukaErrorFromArg(NULL) ) return 1;

  result = unloadmodule_1(NULL, clnt);
  if( setKukaErrorFromResult(result) ) return 1;

  return 0;
}

/*************************************************/
void kuka_getError(kukaVar_t **kukaErrorIn)
{
  kukaVar_t *result=NULL;

  if(kukaErrorIn!=NULL)
    *kukaErrorIn=&kukaError;
  else{
    result = getcrosserror_1(NULL, clnt);
    setKukaErrorFromResult(result);
  }
  return;
}

/*************************************************/
void kuka_displayVar(kukaVar_t *var)
{
  if( var==NULL ){
    printf("Le param�tre de kuka_displayVar ne peut etre NULL.\n");
    return ;
  }

  printf("nom    = \"%s\"\n",var->nom);
  switch(var->val.type){
  case KUKA_INT:
    printf("type   = KUKA_INT\n");
    printf("val = %d\n",var->val.kukaVal_u.kukaInt);
    break;
  case KUKA_REAL:
    printf("type   = KUKA_REAL\n");
    printf("val = %f\n",var->val.kukaVal_u.kukaReal);
    break;
  case KUKA_BOOL:
    printf("type   = KUKA_BOOL\n");
    printf("val = %d\n",var->val.kukaVal_u.kukaBool);
    break;
  case KUKA_CHAR:
    printf("type   = KUKA_CHAR\n");
    printf("val = '%c'\n",var->val.kukaVal_u.kukaChar);
    break;
  case KUKA_STRING:
    printf("type   = KUKA_CHAR\n");
    printf("val = \"%s\"\n",var->val.kukaVal_u.kukaString);
    break;
  case KUKA_AXIS:
    printf("type   = KUKA_AXIS\n");
    printf("val = { A1 %f, A2 %f, A3 %f, A4 %f, A5 %f, A6 %f}\n",\
	   var->val.kukaVal_u.kukaAxis.a1,\
	   var->val.kukaVal_u.kukaAxis.a2,\
	   var->val.kukaVal_u.kukaAxis.a3,\
	   var->val.kukaVal_u.kukaAxis.a4,\
	   var->val.kukaVal_u.kukaAxis.a5,\
	   var->val.kukaVal_u.kukaAxis.a6);
    break;
  case KUKA_FRAME:
    printf("type   = KUKA_FRAME\n");
    printf("val = { X %f, Y %f, Z %f, A %f, B %f, C %f}\n",\
	   var->val.kukaVal_u.kukaFrame.x,\
	   var->val.kukaVal_u.kukaFrame.y,\
	   var->val.kukaVal_u.kukaFrame.z,\
	   var->val.kukaVal_u.kukaFrame.a,\
	   var->val.kukaVal_u.kukaFrame.b,\
	   var->val.kukaVal_u.kukaFrame.c);
    break;
  case KUKA_POS:
    printf("type   = KUKA_POS\n");
    printf("val = { X %f, Y %f, Z %f, A %f, B %f, C%f, S %d, T %d}\n",\
	   var->val.kukaVal_u.kukaPos.x,\
	   var->val.kukaVal_u.kukaPos.y,\
	   var->val.kukaVal_u.kukaPos.z,\
	   var->val.kukaVal_u.kukaPos.a,\
	   var->val.kukaVal_u.kukaPos.b,\
	   var->val.kukaVal_u.kukaPos.c,\
	   var->val.kukaVal_u.kukaPos.s,\
	   var->val.kukaVal_u.kukaPos.t);
    break;
  case KUKA_ERROR:
    printf("type   = KUKA_ERROR\n");
    printf("val = \n");
    printf("\t type = ");
    switch(var->val.kukaVal_u.kukaError.type){
    case KUKA_API_E:
      printf("KUKA_API_E ");
      break;
    case KUKA_RPC_E:
      printf("KUKA_RPC_E ");
      break;
    case KUKA_CROSS_E:
      printf("KUKA_CROSS_E ");
      break;
    case KUKA_KUKA_E:
      printf("KUKA_KUKA_E ");
      break;
    }
    printf("\n\t n� = %ld\n\t description = %s\n",
	   var->val.kukaVal_u.kukaError.no,
	   var->val.kukaVal_u.kukaError.desc);
    break;

  case KUKA_UNKNOWN :
  default :
    printf("type   = KUKA_UNKNOWN\n");
    break;
  }
  return;
}

/*************************************************/
/*  FONCTIONS PRIV�ES                            */
/*************************************************/
/*! Ces fonctions permettent de manager les erreurs
 *  � partir d'un r�ultat.
 *  Cette fonction devrait etre la seule � �crire
 *  dans kukaError pour simplifier la maintenance 
 *  du code.
 */
int setKukaErrorFromArg(void *arg)
{
  if( clnt==NULL ){
    kukaError.val.type=KUKA_ERROR;
    kukaError.val.kukaVal_u.kukaError.type=KUKA_API_E;
    kukaError.val.kukaVal_u.kukaError.no=0;
    strncpy(kukaError.val.kukaVal_u.kukaError.desc,
	    "The api should be initialized. Call kuka_initialize().",
	    KUKA_STRING_LEN);
    return 1;
  }

  if( arg==NULL ){
    kukaError.val.type=KUKA_ERROR;
    kukaError.val.kukaVal_u.kukaError.type=KUKA_API_E;
    kukaError.val.kukaVal_u.kukaError.no=0;
    strncpy(kukaError.val.kukaVal_u.kukaError.desc,
	    "Argument can't be NULL.",
	    KUKA_STRING_LEN);
    return 1;
  }

  return 0;
}

int setKukaErrorFromResult(void *result)
{

  /* GESTION DES ERREURS CONCERNANT clnt_create */
  // erreur lors de la connection au serveur (rpc)
  if( result==(CLIENT*)NULL && clnt == NULL  ){
    kukaError.val.type=KUKA_ERROR;
    kukaError.val.kukaVal_u.kukaError.type=KUKA_RPC_E;
    kukaError.val.kukaVal_u.kukaError.no=0;
    strncpy(kukaError.val.kukaVal_u.kukaError.desc,
	    clnt_spcreateerror(serveur), 
	    KUKA_STRING_LEN);
    return 1;
  }
  // pas d'erreur lors de la connection au serveur (rpc) 
  else if( result!=NULL && result==clnt ){
    return 0;
  }
  /* GESTION DES ERREURS CONCERNANT rpc_call */
  // erreur au niveau de l'appel � une procedure distante (rpc)
  else if( result==(kukaVar_t *)NULL ){
    kukaError.val.type=KUKA_ERROR;
    kukaError.val.kukaVal_u.kukaError.type=KUKA_RPC_E;
    kukaError.val.kukaVal_u.kukaError.no=0;
    strncpy(kukaError.val.kukaVal_u.kukaError.desc,
	    clnt_sperror(clnt,serveur),
	    KUKA_STRING_LEN);
    return 1;
  }
  /* GESTION DES ERREURS DES COUCHES INFERIEURS */
  // erreur d'une des couches inferieures (cross ou kuka)
  else if( ((kukaVar_t *)result)->val.type == KUKA_ERROR ){
    memcpy(&kukaError,(kukaVar_t *)result,sizeof(kukaVar_t)); //devrait aussi marcher
    //kukaError = *result; //devrait aussi marcher
    return 1;
  }
  // pas d'erreur
  else{
    return 0;
  }
  return 0;
}
