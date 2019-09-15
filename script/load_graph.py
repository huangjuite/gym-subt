
import argparse
import tensorflow as tf


def load_graph(frozen_graph_filename):
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    with tf.Graph().as_default() as graph:
        tf.import_graph_def(
            graph_def,
            input_map=None,
            return_elements=None,
            op_dict=None,
            producer_op_list=None
        )
    return graph


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", default="../result/dqn/frozen_model.pb",
                        type=str, help="Frozen model file to import")
    args = parser.parse_args()
    graph = load_graph(args.f)

    for op in graph.get_operations():
        print(op.name, op.values())
    
    
    x = graph.get_tensor_by_name('eval_net/input/s:0')
    y = graph.get_tensor_by_name('eval_net/l3/output_q:0')

        
    # with tf.Session(graph=graph) as sess:
    #     y_out = sess.run(y, feed_dict={
    #         x: [[3, 5, 7, 4, 5, 1, 1, 1, 1, 1]] # < 45
    #     })
    #     print(y_out) # [[ 0.]] Yay!
    # print ("finish")