## Entregables del proyecto “Hacia los autos autónomos en calles de México: manejo a la defensiva en presencia de conducción imprudente, peatones y baches”

Este repositorio contiene los avances de este proyecto: códigos fuente, videos e imágenes de ejemplo del desempeño de lo desarrollado. El software desarrollado se puede agrupar en cinco categorías:

* Desarrollo de un simulador utilizando Gazebo para facilitar el trabajo remoto.
* Detección de carriles
* Control para seguimiento de carriles
* Detección de semáforos
* Detección de personas.

### Simulación en Gazebo
Este simulador está basado en el trabajo desarrollado por el grupo Eagle Knights del ITAM (https://github.com/ITAM-Robotica/Eagle_Knights-Wiki/wiki). A este simulador se le corrigió la simulación de la nube de puntos y se agregaron varios elementos como señales de tránsito, semáforos y peatones. La siguiente imagen muestra un ejemplo del ambiente desarrollado.
<img src="https://github.com/mnegretev/UPV-SelfDrivingCars/blob/master/Media/GazeboExample.png" alt="Detección de peatones" width="640"/>

### Detección de carriles
La detección se realizó mediante detección de bordes (detector de Canny) seguido de segmentación de líneas con Transformada Hough. Para segmentar sólo las líneas de interés, se fijó una región de interés (ROI) triangular donde se espera que las líneas sean observadas. No se segmentas curvas ya que el radio de curvatura es lo suficientemente grande para aproximarlas por rectas de menor longitud. Además, con el control para seguimiento de carril, las líneas siempre aparecerán casi en la misma posición con respecto al vehículo. La siguiente imagen muestra un ejemplo de dicha detección.
<img src="https://github.com/mnegretev/UPV-SelfDrivingCars/blob/master/Media/LaneDetection.png" alt="Detección de peatones" width="1080"/>

### Control para seguimiento de carriles
Se desarrolló un paquete para el seguimiento de carriles. El control es de tipo proporcional y se usan como señales de error la inclinación de los carriles detectados y la distancia de estos con respecto al vehículo. En la siguiente liga se muestra un video del desempeño del controlador.

[Ejemplo del desempeño del controlador](https://drive.google.com/file/d/1eyLQQYxGD0uKTmBdWvZo3rQSjV0Zq0Do/view?usp=sharing)

### Detección de semáforos
Para la detección de semáforos se utilizó una segmentación por color seguida de una Transformada Hough para segmentar círculos. La siguiente imagen muestra un ejemplo del desempeño de este programa:
<img src="https://github.com/mnegretev/UPV-SelfDrivingCars/blob/master/Media/TrafficLightDetection.png" alt="Detección de peatones" width="640"/>

### Detección de peatones
Para la detección de peatones se optó por utilizar una red neuronal profunda implementada con la biblioteca YOLO. Las imágenes de entranamiento se generaron a partir del simulador. Se espera poder usar Learning Transfer para reentrenar la red usando imágenes reales de peatones. La siguiente imagen muestra un ejemplo del resultado de esta detección.
<img src="https://github.com/mnegretev/UPV-SelfDrivingCars/blob/master/Media/PedestrianDetection.png" alt="Detección de peatones" width="640"/>
