# import the inspect_checkpoint library
from tensorflow.python.tools import inspect_checkpoint as chkp

# print all tensors in checkpoint file
chkp.print_tensors_in_checkpoint_file("../result/dqn/subt_rl.ckpt-10000", tensor_name='', all_tensors=False)
