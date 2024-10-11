/graphs: tem o script para criar os gráficos
publish_amcl_path.py: necessário, publica o topico com path do amcl
publish_ekf_pose.py: necessário, substitui o publish_path_odom.py. publica os topicos com a pose estimado ekf e path estimado do ekf
publish_gt.py necessário, publica o topico com a pose do gt
publish_gtPath.py necessário, publica o topico com a path do gt
publish_path_odom.py **not necessário**, o publish_ekf_pose.py encapsulou-o
retrieve_values.py necessário, guarda os valores da bag num .npz file