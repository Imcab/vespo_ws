// =============================================================================
// @NODE@: un modo de vuelo PROPIO de PX4, "@TITLE@", más un ejecutor que lo
// usa para una misión completa. Usa px4-ros2-interface-lib (px4_ros2_cpp).
//
// A diferencia de OFFBOARD, el modo se registra en PX4 como cualquier otro:
// aparece en la lista de modos de QGroundControl, PX4 vigila sus failsafes y
// se puede cambiar de modo con el RC sin que el nodo tenga que enterarse.
//
//   Modo     -> qué hacer en el aire: aquí, un cuadrado de `side` metros
//               alrededor de donde se activó, mirando hacia donde avanza.
//   Ejecutor -> la secuencia completa: armar, despegar (modo Takeoff de PX4),
//               nuestro modo, aterrizar (Land) y esperar a desarmar.
//
// Al elegir "@TITLE@" en QGroundControl (aunque esté desarmado) arranca el
// ejecutor y vuela la misión entera.
//
// OJO: ARMA el vehículo. Pruébalo primero en simulación (vespo up --sim) y en
// el dron real sin hélices.
//
// Ejecutar (con PX4 y el agente corriendo, p. ej. vespo up):
//   ros2 run @PKG@ @NODE@
//   ros2 run @PKG@ @NODE@ --ros-args -p side:=8.0
// Basado en los ejemplos goto y mode_with_executor de px4-ros2-interface-lib.
// =============================================================================

#include <array>
#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include "rclcpp/rclcpp.hpp"

// Nombre con el que PX4 y QGroundControl muestran el modo (< 25 caracteres)
static const std::string kModeName = "@TITLE@";

// -----------------------------------------------------------------------------
// El modo: qué hace el dron mientras está activo
// -----------------------------------------------------------------------------
class @CLASS@Mode : public px4_ros2::ModeBase
{
public:
  explicit @CLASS@Mode(rclcpp::Node & node)
  : ModeBase(node, Settings{kModeName})
  {
    side_ = node.declare_parameter("side", 5.0);             // m, lado del cuadrado
    tolerance_ = node.declare_parameter("tolerance", 0.3);   // m

    // Tipos de setpoint y datos que usa el modo. La librería se encarga de
    // los topics (con su versión) y del heartbeat.
    goto_ = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  // Se llama cada vez que PX4 activa el modo
  void onActivate() override
  {
    // Esquinas del cuadrado en NED (x norte, y este), desde donde estamos
    const Eigen::Vector3f start = position_->positionNed();
    const float s = static_cast<float>(side_);
    corners_ = {start + Eigen::Vector3f{s, 0.f, 0.f}, start + Eigen::Vector3f{s, s, 0.f},
      start + Eigen::Vector3f{0.f, s, 0.f}, start};
    current_ = 0;
    RCLCPP_INFO(node().get_logger(), "%s activo: cuadrado de %.1f m", kModeName.c_str(), side_);
  }

  void onDeactivate() override {}

  // Se llama periódicamente mientras el modo está activo (dt_s = segundos
  // desde la última llamada). Aquí va la lógica de vuelo.
  void updateSetpoint(float /*dt_s*/) override
  {
    const Eigen::Vector3f & target = corners_[current_];
    const Eigen::Vector2f to_target = target.head(2) - position_->positionNed().head(2);

    // Mira hacia donde avanza; cerca del punto el ángulo no está definido
    if (to_target.norm() > 1.f) {
      goto_->update(target, std::atan2(to_target.y(), to_target.x()));
    } else {
      goto_->update(target);
    }

    if ((target - position_->positionNed()).norm() < tolerance_) {
      RCLCPP_INFO(node().get_logger(), "Esquina %zu/4", current_ + 1);
      if (++current_ == corners_.size()) {
        completed(px4_ros2::Result::Success);   // el ejecutor sigue con lo siguiente
      }
    }
  }

private:
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> goto_;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> position_;
  std::array<Eigen::Vector3f, 4> corners_;
  size_t current_ = 0;
  double side_;
  double tolerance_;
};

// -----------------------------------------------------------------------------
// El ejecutor: la secuencia completa alrededor del modo
// -----------------------------------------------------------------------------
class @CLASS@Executor : public px4_ros2::ModeExecutorBase
{
public:
  explicit @CLASS@Executor(px4_ros2::ModeBase & owned_mode)
  // ActivateAlways: se puede elegir el modo estando desarmado y el ejecutor arma
  : ModeExecutorBase(Settings{}.activate(Settings::Activation::ActivateAlways), owned_mode),
    node_(owned_mode.node())
  {
  }

  enum class State { Arming, TakingOff, MyMode, Landing, WaitUntilDisarmed };

  void onActivate() override
  {
    runState(State::Arming, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    if (reason == DeactivateReason::FailsafeActivated) {
      RCLCPP_WARN(node_.get_logger(), "Failsafe: PX4 tomó el control, misión abortada");
    }
  }

  // Cada paso arranca el siguiente cuando termina bien (callback con el resultado)
  void runState(State state, px4_ros2::Result previous)
  {
    if (previous != px4_ros2::Result::Success) {
      RCLCPP_ERROR(node_.get_logger(), "Paso %d falló: %s", static_cast<int>(state),
                   resultToString(previous));
      return;
    }
    switch (state) {
      case State::Arming:
        RCLCPP_INFO(node_.get_logger(), "Armando");
        arm([this](px4_ros2::Result r) {runState(State::TakingOff, r);});
        break;
      case State::TakingOff:
        RCLCPP_INFO(node_.get_logger(), "Despegando (altura: parámetro MIS_TAKEOFF_ALT de PX4)");
        takeoff([this](px4_ros2::Result r) {runState(State::MyMode, r);});
        break;
      case State::MyMode:
        scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {runState(State::Landing, r);});
        break;
      case State::Landing:
        RCLCPP_INFO(node_.get_logger(), "Aterrizando");
        land([this](px4_ros2::Result r) {runState(State::WaitUntilDisarmed, r);});
        break;
      case State::WaitUntilDisarmed:
        waitUntilDisarmed([this](px4_ros2::Result r) {
            RCLCPP_INFO(node_.get_logger(), "Misión terminada (%s)", resultToString(r));
          });
        break;
    }
  }

private:
  rclcpp::Node & node_;
};

// NodeWithModeExecutor crea el nodo, el modo y el ejecutor y los registra en PX4
using @CLASS@Node = px4_ros2::NodeWithModeExecutor<@CLASS@Executor, @CLASS@Mode>;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<@CLASS@Node>("@NODE@", false));   // false: sin logs de depuración
  rclcpp::shutdown();
  return 0;
}
