#ifndef KUKALIB_H
#define KUKALIB_H

#include <kuka_driver/kuka_api/kuka_api.h>

namespace kukalib
{
/**
 *  Libreria principal de control robot KUKA.
 *  
 */
class Kuka {
private:
  // Variables KUKA KRL
  kukaVar_t axisGoal, axisAct, ovPro, moveId, moveFlag, stopFlag;
  // Punteros
  kukaAxis_t *axisGoalPtr, *axisActPtr;
  kukaInt_t *moveIdPtr;
  kukaBool_t *moveFlagPtr, *stopFlagPtr;

  float current_joint_[6];
  bool staticPose;

public:
  Kuka();

  ~Kuka();
  /**
  * Inicializar la conexión con el robot KUKA
  * @param server IP o nombre del PC del robot KUKA. Debe estar en la misma red. Puede editar el archivo /etc/hosts para dar un nombre.
  */
  bool beginConnection(std::string server);

  /**
  * Cierra la conexion con el robot KUKA.
  */
  void endConnection();

  /**
   * Mueve los ejes del robot KUKA a la posicion Home.
   * (KUKA 0, -90, 90, 0, 0, 0) (ROS 0, 0, 0, 0, 0, 0)
   * @return Retorna true si no existieron problemas, false en caso contrario.
   */
  bool goHome();
  /**
  * Frena el robot.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  bool stop();

  /**
  * Actualiza los valores de los ejes y estado de movimiento.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  bool update();

  /**
  * Configura la velocidad override del robot KUKA.
  * @param vel Velocidad Override en porcentaje. Para una operacion segur se recomienda no superar el 40%.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  bool setOverrideSpeed(int vel);

  /**
  * Mueve los ejes del robot KUKA. Usando grados y offset originales del robot, forma directa de movimiento PTP.
  * @param a1 Posicion del eje A1 en grados.
  * @param a2 Posicion del eje A2 en grados.
  * @param a3 Posicion del eje A3 en grados.
  * @param a4 Posicion del eje A4 en grados.
  * @param a5 Posicion del eje A5 en grados.
  * @param a6 Posicion del eje A6 en grados.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  bool setJointPosition(float a1, float a2, float a3, float a4, float a5, float a6);
  bool setJointPosition(float (&joint)[6]);
  
  /**
  * Mueve los ejes del robot KUKA. Usando radianes y offset del modelo de ROS.
  * @param joint Mensaje ROS que contiene la posicion deseada.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  //bool setJointPosition(uchile_kuka::KukaJoint &joint);
  bool setEfectorPosition(float pos[]);

  /**
  * Envia una variable al robot KUKA.
  * @param var Variable.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  bool setVariable(kukaVar_t *var);

  /**
  * Obtiene una variable del robot KUKA.
  * @param var Variable.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  bool getVariable(kukaVar_t *var);

  bool getMoveFlag(bool &move);
  /**
  * Retorna la posicion del joint i
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  float getJointPosition(int i);
  void getJointPosition(float (&joint)[6]);
  void getJointPosition(std::vector<double> &joint);


  /**
  * Obtiene posicion de los joints del robot KUKA.
  * @param joint Variable para escriir los joints.
  * @return Retorna true si no existieron problemas, false en caso contrario.
  */
  //bool getJointPosition(uchile_kuka::KukaJointState &joint);
  bool getState();
};

} // kukalib namespace

#endif
