# Estados

- Takeoff (stdstates)
- SearchBall
- GoToBall
- Rise
- Align (with wire) (stdstates)
- DropTheHook
- GoTo (base) (stdstates)
- Landing (stdstates)

## Takeoff
Estado de decolagem do drone. Contido em stdstates.

## SearchBall
Existem duas mangueiras no entorno do drone. Apenas uma delas possui uma bola (a ser encontrada).
Este estado faz a varredura em yaw a fim de encontrar a bola.
(Ainda é necessário pensar em um plano B para caso a bola não ser encontrada apenas com um giro de 360º)

## GoToBall
As imagens da câmera horizontal são antes processadas por um nó de ROS2 a fim de obter algumas informações sobre a bola a ser detectada (posição do centro e estimativa de distância, por exemplo). Essas informações já analisadas serão recebidas por esse estado para que o Drone vá até a bola.
A posição do centro da bola alimentará o PID (PIDController contido no drone_lib) para que o drone siga efetivamente essa bola.
A estimativa de distância recebida será considerada para ativar o gatilho, baseada em um parâmetro configurável via yaml, para sair do estado de seguir a bola e ir para o estado de Rise.

## Rise
Estado responsável por elevar a altura do drone em poucos metros (delta de altura configurável via yaml).

## Align (with wire)
Utiliza o estado Align do stdstates para realizar um alinhamento preciso com a mangueira (centro e yaw).

## DropTheHook
Estado responsável por invocar um script python para realizar a soltura do gancho sobre a mangueira.

## GoTo (base)
Utiliza o estado GoTo do stdstates para retornar para a base (ponto conhecido)

## Landing
Estado de pouso do drone. Contido em stdstates.