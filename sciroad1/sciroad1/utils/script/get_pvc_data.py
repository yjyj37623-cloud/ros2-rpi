
def get_batch_data(sampling):
    for i in range(5):
        name = '_NO' + str(i + 1)
        sampling.goback_step(save_name=name)
