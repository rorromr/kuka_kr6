/*! \mainpage Kuka API.
 *  \include VERSION
 *  \include README
 */

#define RELEASE 0.1.0

/*! \file
 *  Ce fichier d�crit l'interface d'utilisation de kuka. Cette
 *  interface d�finit des fonctions de "bas-niveaux", et un 
 *  rapport d'erreur permettant de construire par dessus une interface 
 *  dont les fonctions seraient plus sp�cifiques ( par exemple move,
 *  openGripper, ...).
 */

/****************************\
  TYPES
\****************************/
/*! \name Constantes.
 */
//@{

/*! Ces macro devraient etre utilis�es pour affecter aux
 *  variables de type kukaBool_t la valeur "TRUE".
 */
#define TRUE 1
/*! Ces macro devraient etre utilis�es pour affecter aux
 *  variables de type kukaBool_t la valeur "FALSE".
 */
#define FALSE 0

/*! Les variables, en KRL font 12 charact�res maximum.
 *  Ajoutons � cela un charact�re de fin de chaine
 *  et un pour la forme, et nous en avons donc 14.
*/
#define KUKA_VARNAME_LEN 14

/*! Nous dirons que les chaines de charact�res aurons
 *  une longeur de 128. Ces chaines contiennent :
 *  - les valeurs des variables de type kukaString_t
 *  ($mode_op -> "#EX" par exemple).
 *  - les descriptions des erreurs de type kukaErrorString_t.
 */
#define KUKA_STRING_LEN 128

//@}

/****************************/
/*! \name Types simples.
 */
//@{

/*! Les 'real' renvoy�s par kuka seront interpr�t�s comme
 *  des 'float' en c.
*/
typedef float kukaReal_t;

/*! Les 'int' renvoy�s par kuka seront interpr�t�s comme 
 *  des 'int' en c. 
 */
typedef int kukaInt_t;

/*! Les 'bool' renvoy�s par kuka seront interpr�t�s comme 
 *  des 'int' en c. */
typedef int kukaBool_t;

/*! Les 'char' simples n'existent pas en krl, il y a seulement
 *  des chaines de charact�res.
 *  \sa kukaString_t
 */
typedef char kukaChar_t;

/*! Les chaines de charact�res du KRL seront interpr�t�s
 *  comme des tableaux de char en c.
 */
typedef kukaChar_t kukaString_t[KUKA_STRING_LEN];

//@}

/****************************/
/*! \name Types structur�es.
 */
//@{

/*! Structure permettant de d�crire une position 
 *  sp�cifique aux axes. Ici, comme chaque axe est un pivot,
 *  les composantes de cette structure sont des angles.
 *  \brief Description d'une configuration en fonction de chaque articulation.
 *  \image html kukaAxis_t.jpg
 *  \todo impl�menter le type E6AXIS
*/
struct kukaAxis_s
{
  /*! Rotation autour de l'axe 1 */
  kukaReal_t a1;
  /*! rotation autour de l'axe 2 */
  kukaReal_t a2;
  /*! rotation autour de l'axe 3 */
  kukaReal_t a3;
  /*! rotation autour de l'axe 4 */
  kukaReal_t a4;
  /*! rotation autour de l'axe 5 */
  kukaReal_t a5;
  /*! rotation autour de l'axe 6 */
  kukaReal_t a6;
};
typedef struct kukaAxis_s kukaAxis_t;

/*! Cette structure repr�sente le type de donn�e 'frame' du krl.
 *  Elle permet de repr�senter l'orientation et la position
 *  d'un point dans l'espace.<br>
 *  Les composantes a,b et c repr�sentent le roulis, le tangage et 
 *  l'embard�e. Plus pr�cis�ment : 
 *  - a est l'angle de rotation autour de Z,
 *  - b est l'angle de rotation autour de Y,
 *  - c est l'angle de rotation autour de X.
 *  \image html kukaFrame_t.jpg
 *  \brief Description d'un point (position/orientation) dans l'espace.  
 */
struct kukaFrame_s
{
  /*! Composante x des coordonn�es cart�siennes du point. */
  kukaReal_t x;
  /*! Composante y des coordonn�es cart�siennes du point. */
  kukaReal_t y;
  /*! Composante z des coordonn�es cart�siennes du point. */
  kukaReal_t z;
  /*! Angle de rotation autour de z */
  kukaReal_t a;
  /*! Angle de rotation autour de y */
  kukaReal_t b;
  /*! Angle de rotation autour de x */
  kukaReal_t c;
};
typedef struct kukaFrame_s kukaFrame_t;

/*! Cette structure repr�sente le type de donn�e 'pos' du krl.
 *  Bien que le type kukaFrame_t soit suffisant et sans equivoque
 *  pour d�crire des coordonn�es, il y a parfois plusieurs configuration
 *  d'axe possibles pour atteindre un point. Donc grace � s et t on l�ve
 *  l'ambiguit�. Pour une description d�taill�e des membres x,y,z,a,b et c
 *  de la structure voir kukaFrame_s.
 *  \sa kukaFrame_s
 *  \brief Description sans equivoque d'un point (position/orientation).  
 *  \todo impl�menter le type E6POS
 */
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
typedef struct kukaPos_s kukaPos_t;

//@}

/****************************/
/*! \name Enum�ration des types de variables.
 */
//@{

/*! Cette enum�ration permet de lister les types dont nous disposons.
 *  De 10 � 19 nous avons les types simples
 *  De 20 � 29 nous avons les types structures
 *  De
 */
enum kukaType_e{
  /*---------------------------*/
  KUKA_UNKNOWN          = 0x0000 | 0x000,
  /*---------------------------*/
  /*! Les types simples sont les kukaInt_t, kukaReal_t, kukaBool_t,
   *  kukaChar_t et les kukaString_t */
  KUKA_SIMPLE           = 0x0000 | 0x0001,
  KUKA_INT              = 0x0010 | KUKA_SIMPLE,
  KUKA_REAL             = 0x0020 | KUKA_SIMPLE,
  KUKA_BOOL             = 0x0030 | KUKA_SIMPLE,
  KUKA_CHAR             = 0x0040 | KUKA_SIMPLE,
  KUKA_STRING           = 0x0050 | KUKA_SIMPLE,
  /*---------------------------*/
  /*! Les types structure sont les : kukaAxis_s, kukaFrame_s, kukaPos_s */
  KUKA_STRUCT           = 0x0000 | 0x0002,
  KUKA_AXIS             = 0x0010 | KUKA_STRUCT,
  KUKA_FRAME            = 0x0020 | KUKA_STRUCT,
  KUKA_POS              = 0x0030 | KUKA_STRUCT,
  /*---------------------------*/
  KUKA_ERROR            = 0x0000 | 0x000a,
};
typedef enum kukaType_e kukaType_t;

//@}

/****************************/
/*! \name D�clarations relatives aux erreurs.
 */
//@{

/*! Enum�ration des diff�rents types d'erreur. */
enum kukaErrorType_e {
  /*! Ces erreur sont g�n�r�es par le robot (cot� serveur donc). */
  KUKA_KUKA_E           = 0x0010 | KUKA_ERROR,
  /*! Ces erreur sont g�n�r�es par les appels de fonctions 
   *  d'acc�s au composant com crosscommexe (cot� serveur donc). */
  KUKA_CROSS_E          = 0x0020 | KUKA_ERROR,
  /*! Ces erreur sont g�n�r�es par les appels de fonctions 
   *  rpc (cot� client donc). */
  KUKA_RPC_E            = 0x0030 | KUKA_ERROR,
  /*! Ces erreur sont g�n�r�es par les appels de fonctions 
   *  de l'api (cot� client donc). */
  KUKA_API_E            = 0x0040 | KUKA_ERROR
};
typedef enum kukaErrorType_e kukaErrorType_t;

/*! Grace � cette structure les erreurs sont d�crites en 
 *  fonction de :
 *  - leur provenance (leur type).
 *  - leur num�ro (qui pour l'instant sert surtout pour 
 *  r�cup�rer la description des erreurs de type KUKA_KUKA_E).
 *  - leur description.
 * \brief Description des erreurs.
 */
struct kukaError_s
{
  /*! Le type d'erreur d�pend de sa source. Si l'erreur est g�n�r�e par :
   *  - le robot (cot� serveur donc) : type =  KUKA_KUKA_E
   *  - les appels de fonctions d'acc�s au composant com crosscommexe 
   *    (cot� serveur donc) : type =  KUKA_CROSS_E
   *  - les appels de fonctions rpc (cot� client donc) : type =  KUKA_RPC_E
   *  - les appels de fonctions de l'api (cot� client donc) : type =  KUKA_API_E
   */
  kukaErrorType_t type;
  /*! Ce num�ro sert, pour l'instant, surtout � r�cup�rer la description
   *  des erreurs de type KUKA_KUKA_E.
   */
  long no;
  /*! La description. Tant que possible elle a �t� faite de mani�re suivant :
   *  "source: description"
   */
  kukaString_t desc;
};
typedef struct kukaError_s kukaError_t;

//@}

/****************************/
/*! \name D�finition de la composition d'une variable.
 */
//@{

/*! Union des valeurs
 *  Si type=KUKA_XYZ alors kukaVal_u.kukaXyz est de type kukaXyz_t.
 *
 *  Par exemple si nous voulons une variable de type position
 *  nous aurons :
 *  \code
 *  variable.nom="$POS_ACT";
 *  variable.valeur.type=KUKA_POS;
 *  variable.valeur.kukaVal_u.kukaPos.x=(kukaFloat_t)3.5;
 *  \endcode
 *  \brief Union de valeurs typ�es.  
 */
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

//@}

/****************************/
/*! \name D�finition d'une variable.
 */
//@{

/*! Structure d�finissant une variable. 
 *  \brief D�finition d'une variable.
*/
struct kukaVar_s
{
  /*! Nom de la variable */
  char nom[KUKA_VARNAME_LEN];
  /*! Valeur de la variable */
  kukaVal val;
};
typedef struct kukaVar_s kukaVar_t;

//@}


/****************************\
  FONCTIONS
\****************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! \name Fonctions de base
 *  Ces fonctions retournent 1 si elles ont g�n�r� une erreur. L'erreur
 *  est consultable via la variable statique kukaError de la lib kuka_api.
 *  \note Un pointeur sur kukaError s'obtient en invoquant kuka_getError.
 */
//@{
    
/*! kuka_initialize :
 *  - se connecte au serveur rpc,
 *  - invoque l'initialisation du cross (qui initialise COM),
 *  - invoque la connection au cross.
 *
 *  \param serveur est un pointeur vers la chaine de charact�res contenant 
 *  le nom du serveur ("kuka" g�n�ralement).
 *
 *  \return 1 si �chec.
 *  \return 0 si succ�s.
 */
int kuka_initialize(char* serveur);

/*! kuka_uninitialize permet simplement de tout finaliser et de tout 
 *  d�connecter.
 *
 *  \return 1 si �chec.
 *  \return 0 si succ�s.
 */
int kuka_uninitialize(void);

/*! kuka_getVar permet de r�cup�rer la valeur d'une variable en fonction de 
 *  son nom et de son type.
 *
 *  \param varInOut est un pointeur vers la variable.
 *
 *  Les pr�-conditions pour cette variable sont :
 *  - le champ nom contient le nom de la variable.
 *  - le champ valeur.type contient le type de la varable.
 *
 *  Les post-conditions pour cette variable sont :
 *  - en cas de succ�s le champ valeur.kukaVal_u.kuka*** contient la valeur
 *  - en cas d'echec varInOut contient une copie de kukaError.
 *
 *  \return 1 si �chec.
 *  \return 0 si succ�s.
 */
int kuka_getVar(kukaVar_t *varInOut);

/*! kuka_setVar permet de d�finir la valeur d'une variable en fonction de 
 *  son nom et de son type.
 *
 *  \param varIn est un pointeur vers la variable.
 *
 *  Les pr�-condition 
 *  pour cette variable sont :
 *  - le champ nom contient le nom de la variable.
 *  - le champ valeur.type contient le type de la varable.
 *
 *  Les Post-conditions pour cette variable sont : *varIn reste inchang�.
 *
 *  \return 1 si �chec.
 *  \return 0 si succ�s.
 */
int kuka_setVar(kukaVar_t *varIn);

/*! \attention Ne pas utiliser pour l'instant 
*/
int kuka_loadModule(char* module);

/*! \attention Ne pas utiliser pour l'instant 
*/
int kuka_unloadModule(void);

//@}

/****************************\
  UTILS
\****************************/
/*! \name Fonctions utilitaires
 */
//@{

/*! Cette fonction permet d'afficher les informations d'une variable
 *  sur la sortie standard.
*/
void kuka_displayVar(kukaVar_t *var);

/*! Cette fonction a une double utilit� :
 *  - elle affecte au pointeur kukaError l'adresse d'une
 *    variable d�clar�e statique dans la lib kuka_api. Grace � ce 
 *    pointeur nous pouvons consulter un "rapport d'erreur".
 *  - elle met � jour le "rapport d'erreur".
 *
 *  \sa kukaError_s
 *  \param kukaError==NULL la fonction met � jour le rapport d'erreur.
 *  \param kukaError!=NULL r�cup�re le rapport d'erreur
 */
void kuka_getError(kukaVar_t **kukaError);

//@}

#ifdef __cplusplus
}
#endif

