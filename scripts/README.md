/graphs: tem o script para criar os gráficos
amcl_npz_retriever.py: guarda os valores de amcl num npz
ekf_npz_retriever.py: guarda os valores de ekf num npz
publish_amcl_path.py: necessário, publica o topico com path do amcl (usado na bag gravada)
publish_ekf_error.py: publica erro euclideano do ekf
publish_gt.py necessário, publica o topico com a pose do gt
publish_gtPath.py necessário, publica o topico com a path do gt
publish_path_odom.py **not necessário**, o transformed_ekf.py encapsulou-o
retrieve_values_amcl.py parte 2 do retriever mas c mais cenas
retrieve_values_ekf.py parte 2 do retriever mas c mais cenas
transformed_ekf.py: necessário, substitui o publish_path_odom.py. publica os topicos com a pose estimado ekf e path estimado do ekf transformados
transformed_amcl.py: publica os topicos com a pose estimado amcl e path estimado do amcl transformados