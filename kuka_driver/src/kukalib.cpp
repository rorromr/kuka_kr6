
#include <ros/ros.h>
#include <kuka_driver/kukalib.h>


namespace kukalib
{

const float PI = 3.14159265358979f, rad2deg = 180.0 / 3.14159265358979,
  deg2rad = 3.14159265358979 / 180.0;

// OOP
Kuka::Kuka() {
  // Inicializar variables KRL
  axisGoal = (kukaVar_t) { "API_MOVE_AXIS", { KUKA_AXIS, { 0 } } };  // Angulos de ejes para movimiento
  moveId = (kukaVar_t) { "API_MOVE_ID", { KUKA_INT, { 10 } } };      // ID de movimiento
  moveFlag = (kukaVar_t) { "API_MOVE_FLAG", { KUKA_BOOL, { 1 } } };  // Habilitar movimiento
  stopFlag = (kukaVar_t) { "API_B_1", { KUKA_BOOL, { 0 } } };        // Mensaje de stop


  // Variables del sistema
  ovPro = (kukaVar_t) { "$OV_PRO", { KUKA_INT, { 10 } } };  // Velocidad Override
  axisAct = (kukaVar_t) { "$AXIS_ACT", { KUKA_AXIS, { 0 } } };  // Posición actual de ejes

  // Punteros a variables
  axisGoalPtr = &axisGoal.val.kukaVal_u.kukaAxis;
  moveIdPtr   = &moveId.val.kukaVal_u.kukaInt;
  moveFlagPtr = &moveFlag.val.kukaVal_u.kukaBool;
  stopFlagPtr = &stopFlag.val.kukaVal_u.kukaBool;
  axisActPtr  = &axisAct.val.kukaVal_u.kukaAxis;

  // Robot estatico
  staticPose = false;
  for (int i = 0; i < 6; ++i) current_joint_[i] = 0;
}

/**
 * Close connection with KUKA robot.
 */
Kuka::~Kuka(){
  // Close conection
  endConnection();
}

/**
 * Inicializar la conexión con el robot KUKA
 * @param server IP o nombre del PC del robot KUKA. Debe estar en la misma red. Puede editar el archivo /etc/hosts para dar un nombre.
 */
bool Kuka::beginConnection(std::string server) {
  char pserver[server.length() + 1];
  strcpy(pserver, server.c_str()); // copy server name
  int error = kuka_initialize(pserver); // Call KUKA API
  // Manejo de errores
  if (error == 1) {
    ROS_ERROR("KUKA initialisation problem.");
    return false;
  } else if (error == 2) {
    ROS_WARN("KUKA seems to be already initialised.");
  }
  ROS_INFO("Succeeded initialisation.");
  return true;
}

/**
 * Close connection with KUKA robot.
 */
void Kuka::endConnection() {
  int error = kuka_uninitialize(); // Call KUKA API
  if (error) {
    ROS_ERROR("KUKA finalization problem.");
  }
  ROS_WARN("Succeeded finalize KUKA robot conection.");
  return;
}

/**
 * Envia una variable al robot KUKA.
 * @param var Variable.
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
bool Kuka::setVariable(kukaVar_t *var) {
  int error = kuka_setVar(var); //Exportar variable
  if (error == 1) {
    ROS_ERROR("[KUKALib] Error setting KUKA variable.");
    return false;
  }
  return true;
}

/**
 * Obtiene una variable del robot KUKA.
 * @param var Variable.
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
bool Kuka::getVariable(kukaVar_t *var) {
  int error=kuka_getVar(var);
  if( error == 1 ){
    ROS_ERROR("[KUKALib] Error getting KUKA variable.");
    return false;
  }
  return true;
}

/**
 * Configura la velocidad override del robot KUKA.
 * @param vel Velocidad Override en porcentaje. Para una operacion segur se recomienda no superar el 40%.
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
bool Kuka::setOverrideSpeed(int vel){
  ovPro.val.kukaVal_u.kukaInt=vel;
  return setVariable(&ovPro);
}

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
bool Kuka::setJointPosition(float a1, float a2, float a3, float a4, float a5, float a6) {
  axisGoalPtr->a1 = a1;
  axisGoalPtr->a2 = a2;
  axisGoalPtr->a3 = a3;
  axisGoalPtr->a4 = a4;
  axisGoalPtr->a5 = a5;
  axisGoalPtr->a6 = a6;
  *moveIdPtr = (kukaInt_t) 1;
  ROS_INFO("KUKA setting joints.");
  return setVariable(&axisGoal) && setVariable(&moveId);
}

bool Kuka::setJointPosition(float (&joint)[6]){
  axisGoalPtr->a1 = joint[0];
  axisGoalPtr->a2 = joint[1];
  axisGoalPtr->a3 = joint[2];
  axisGoalPtr->a4 = joint[3];
  axisGoalPtr->a5 = joint[4];
  axisGoalPtr->a6 = joint[5];
  *moveIdPtr = (kukaInt_t) 1;
  return setVariable(&axisGoal) && setVariable(&moveId);
}

void Kuka::getJointPosition(float (&joint)[6]){
  joint[0] = current_joint_[0];
  joint[1] = current_joint_[1];
  joint[2] = current_joint_[2];
  joint[3] = current_joint_[3];
  joint[4] = current_joint_[4];
  joint[5] = current_joint_[5];
}

void Kuka::getJointPosition(std::vector<double> &joint){
  joint[0] = current_joint_[0];
  joint[1] = current_joint_[1];
  joint[2] = current_joint_[2];
  joint[3] = current_joint_[3];
  joint[4] = current_joint_[4];
  joint[5] = current_joint_[5];
}

/**
 * Mueve los ejes del robot KUKA. Usando radianes y offset del modelo de ROS.
 * @param joint Mensaje ROS que contiene la posicion deseada.
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
 /*
bool Kuka::setJointPosition(uchile_kuka::KukaJoint &joint)
{
  axisGoalPtr->a1 = joint.position[0] * rad2deg;
  axisGoalPtr->a2 = joint.position[1] * rad2deg - 90.0;
  axisGoalPtr->a3 = joint.position[2] * rad2deg + 90.0;
  axisGoalPtr->a4 = joint.position[3] * rad2deg;
  axisGoalPtr->a5 = joint.position[4] * rad2deg;
  axisGoalPtr->a6 = joint.position[5] * rad2deg;

  *moveIdPtr = (kukaInt_t)1;
  
  if (setVariable(&axisGoal) && setVariable(&moveId))
  {
    ROS_INFO("KUKA setting joints.");
    current_joint_.position = joint.position;
    ++current_joint_.header.seq;
    current_joint_.header.stamp = ros::Time::now();
    return true;
  }
  return false;
}
*/

/**
 * Mueve los ejes del robot KUKA a la posicion Home.
 * (KUKA 0, -90, 90, 0, 0, 0) (ROS 0, 0, 0, 0, 0, 0)
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
bool Kuka::goHome() {
  ROS_INFO("KUKA robot go Home.");
  // Set in home position
  return setJointPosition(0, -90, 90, 0, 0, 0);
}

/**
 * Mueve los ejes del robot KUKA a la posicion Home.
 * (KUKA 0, -90, 90, 0, 0, 0) (ROS 0, 0, 0, 0, 0, 0)
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
bool Kuka::stop(){
  *stopFlagPtr = (kukaBool_t) true;
  ROS_WARN("KUKA Stop.");
  return setVariable(&stopFlag);
}

/**
 * Actualiza los valores de los ejes y estado de movimiento.
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
bool Kuka::update(){
  // Update joint positions
  if (getVariable(&axisAct) && getVariable(&moveFlag))
  {
    // Joints
    current_joint_[0] = axisActPtr->a1;
    current_joint_[1] = axisActPtr->a2;
    current_joint_[2] = axisActPtr->a3;
    current_joint_[3] = axisActPtr->a4;
    current_joint_[4] = axisActPtr->a5;
    current_joint_[5] = axisActPtr->a6;
    // Robot estatico
    staticPose = *moveFlagPtr;
    return true;
  }
  return false;
}

/**
 * Retorna la posicion del joint i
 * @return Retorna true si no existieron problemas, false en caso contrario.
 */
float Kuka::getJointPosition(int i){
  return current_joint_[i];
}

bool Kuka::getMoveFlag(bool &move)
{
  if (getVariable(&moveFlag))
  {
    move = (bool)*moveFlagPtr;
    return true;
  }
  else
  {
    return false;
  }
}

/*
bool Kuka::getJointPosition(uchile_kuka::KukaJointState &joint)
{
  if (getVariable(&axisAct))
  {
    // update joints
    current_joint_.position[0] = axisActPtr->a1 * deg2rad;
    current_joint_.position[1] = (axisActPtr->a2 + 90.0) * deg2rad;
    current_joint_.position[2] = (axisActPtr->a3 - 90.0) * deg2rad;
    current_joint_.position[3] = axisActPtr->a4 * deg2rad;
    current_joint_.position[4] = axisActPtr->a5 * deg2rad;
    current_joint_.position[5] = axisActPtr->a6 * deg2rad;

    // update header
    ++current_joint_.header.seq;
    current_joint_.header.stamp = ros::Time::now();
    // update joint
    joint.header=current_joint_.header;
    joint.name=current_joint_.name;
    joint.position=current_joint_.position;
    //Print
    ROS_INFO_STREAM("LIB A1 " << joint.position[0]);
    return true;
  }
  else {
    return false;
  }
}
*/

bool Kuka::getState(){
  return staticPose;
}

/*
 * Funciones para obtener variables del sistema por nombre
 * eg
 * kukaVar_t corriente = { "$CURR_ACT[1]", { KUKA_REAL, { 0 } } };
 *
 *
 */
} //kukalib namespace