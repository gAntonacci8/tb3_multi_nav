import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TfsRepublisher(Node):
    """
    Nodo centrale (senza namespace) che si sottoscrive al TF globale
    e ripubblica le trasformate per i namespace robot1 e robot2.
    """
    def __init__(self):
        super().__init__('tfs_republisher')
        
        # Hardcoding dei namespace dei robot
        self.robot_namespaces = ['robot1', 'robot2']
        
        self.get_logger().info(f"Avvio TF Central Republisher per: {self.robot_namespaces}")
        
        # Dizionari per i publisher namespaced
        self.tf_publishers = {}
        self.tf_static_publishers = {}

        # 1. Crea i publisher namespaced per ogni robot
        for ns in self.robot_namespaces:
            # I publisher puntano a /robotX/tf e /robotX/tf_static
            self.tf_publishers[ns] = self.create_publisher(
                TFMessage, 
                f'/{ns}/tf', 
                100 # QoS history depth
            )
            self.tf_static_publishers[ns] = self.create_publisher(
                TFMessage, 
                f'/{ns}/tf_static', 
                100 # QoS history depth
            )

        # 2. Sottoscrizioni globali (Gazebo/Robot State Publisher)
        # La "/" iniziale assicura che i topic siano risolti a livello globale (/tf)
        self.create_subscription(
            TFMessage,
            '/tf',
            self.dynamic_tf_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.create_subscription(
            TFMessage,
            '/tf_static',
            self.static_tf_callback,
            rclpy.qos.qos_profile_system_default
        )

    def _filter_and_republish(self, msg: TFMessage, publishers: dict):
        """Filtra e ripubblica le trasformate per tutti i namespace definiti."""
        
        # Crea un dizionario di messaggi TF separati per ogni robot
        # Questo è più efficiente che filtrare/pubblicare due volte lo stesso messaggio
        msgs_to_publish = {ns: TFMessage() for ns in self.robot_namespaces}

        for transform in msg.transforms:
            # Identifica a quale robot appartiene la trasformata
            owner_namespace = None

            # Controlla se il frame è namespaced (es. 'robot1/base_link')
            for ns in self.robot_namespaces:
                prefix = f'{ns}/'
                if transform.child_frame_id.startswith(prefix) or transform.header.frame_id.startswith(prefix):
                    owner_namespace = ns
                    break

            # Gestisce le trasformate globali che si connettono ai frame namespaced
            # Esempio: 'map' -> 'robot1/odom' (fornito da AMCL)
            if owner_namespace is None and transform.header.frame_id == 'map':
                for ns in self.robot_namespaces:
                    if transform.child_frame_id == f'{ns}/odom':
                        owner_namespace = ns
                        break

            # Se abbiamo trovato un proprietario, aggiungi la trasformata al suo messaggio
            if owner_namespace:
                msgs_to_publish[owner_namespace].transforms.append(transform)

        # Pubblica i messaggi TF filtrati
        for ns, new_msg in msgs_to_publish.items():
            if new_msg.transforms:
                publishers[ns].publish(new_msg)

    def dynamic_tf_callback(self, msg):
        """Callback per il topic /tf dinamico."""
        self._filter_and_republish(msg, self.tf_publishers)

    def static_tf_callback(self, msg):
        """Callback per il topic /tf_static statico."""
        self._filter_and_republish(msg, self.tf_static_publishers)

def main(args=None):
    rclpy.init(args=args)
    tf_republisher = TfsRepublisher()
    rclpy.spin(tf_republisher)
    tf_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# class TfRepublisher(Node):
#     """
#     Nodo che si sottoscrive ai topic TF globali (/tf e /tf_static)
#     e ripubblica le trasformate relative a un robot specifico
#     sul topic namespaced (/robot_namespace/tf o /robot_namespace/tf_static).
#     """
#     def __init__(self):
#         super().__init__('tf_republisher')

#         # Ottieni il namespace del nodo. Sarà usato per il filtraggio e la pubblicazione.
#         # Es: 'robot1' se lanciato con ros2 run ... --ros-args -r __ns:=/robot1
#         self.namespace = self.get_namespace().strip('/')
        
#         if not self.namespace:
#             self.get_logger().error("Il nodo TF Republisher è stato lanciato senza un namespace. Uscita.")
#             raise Exception("Namespace non specificato.")

#         self.get_logger().info(f"Avvio TF Republisher per il namespace: /{self.namespace}")
        
#         # 1. Pubblicatori namespaced (dove i nodi Nav2 namespaced si aspettano il TF)
#         # Nota: i topic vengono risolti come /<namespace>/tf
#         self.tf_publisher = self.create_publisher(TFMessage, 'tf', 100)
#         self.tf_static_publisher = self.create_publisher(TFMessage, 'tf_static', 100)

#         # 2. Sottoscrizioni globali (da dove provengono i dati di Gazebo/RSP)
#         # La "/" iniziale assicura che i topic siano risolti a livello globale
#         self.create_subscription(
#             TFMessage,
#             '/tf',
#             self.dynamic_tf_callback,
#             rclpy.qos.qos_profile_sensor_data  # Usare qos_profile_sensor_data per messaggi ad alta frequenza
#         )

#         self.create_subscription(
#             TFMessage,
#             '/tf_static',
#             self.static_tf_callback,
#             rclpy.qos.qos_profile_system_default # TF statico è meno urgente
#         )

#     def _filter_and_republish(self, msg: TFMessage, publisher):
#         """Filtra le trasformate che appartengono a questo robot e le ripubblica."""
        
#         # Crea una nuova lista di trasformate filtrate
#         filtered_transforms = []

#         # Il prefisso da cercare (es. 'robot1/')
#         prefix = f'{self.namespace}/'

#         for transform in msg.transforms:
#             # Controllo 1: I frame del robot usano il prefisso (es. robot1/base_link, robot1/odom)
#             if transform.child_frame_id.startswith(prefix) or transform.header.frame_id.startswith(prefix):
#                 filtered_transforms.append(transform)

#             # Controllo 2 (Eccezione): Trasformate globali 'map' o 'odom' che puntano al frame del robot.
#             # Esempio: 'map' -> 'robot1/odom' (da AMCL) o 'odom' -> 'robot1/base_footprint' (da Gazebo)
#             elif (transform.header.frame_id == 'map' and transform.child_frame_id == f'{self.namespace}/odom') or \
#                  (transform.header.frame_id == f'{self.namespace}/odom' and transform.child_frame_id == f'{self.namespace}/base_footprint'):
#                  filtered_transforms.append(transform)

#             # Controllo 3: La trasformata 'map' -> 'odom' di AMCL
#             # Questo è essenziale per il multi-robot. La trasformata è 'map' -> '<namespace>/odom'.
#             elif transform.header.frame_id == 'map' and transform.child_frame_id == f'{self.namespace}/odom':
#                  filtered_transforms.append(transform)
            
#             # Nota: la trasformata 'map'->'<namespace>/odom' sarà filtrata dal check 1 se AMCL è configurato
#             # correttamente, ma è bene includere la logica per chiarezza.

#         if filtered_transforms:
#             new_msg = TFMessage()
#             new_msg.transforms = filtered_transforms
#             publisher.publish(new_msg)

#     def dynamic_tf_callback(self, msg):
#         """Callback per il topic /tf dinamico."""
#         self._filter_and_republish(msg, self.tf_publisher)

#     def static_tf_callback(self, msg):
#         """Callback per il topic /tf_static statico."""
#         self._filter_and_republish(msg, self.tf_static_publisher)

# def main(args=None):
#     rclpy.init(args=args)
#     tf_republisher = TfRepublisher()
#     rclpy.spin(tf_republisher)
#     tf_republisher.destroy_node()
#     rclpy.shutdown()

# if __file__ == '__main__':
#     main()

