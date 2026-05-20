
def get_lstm_edge_surface_data(sampling, ):
    pass
def get_lstm_one_surface_data(sampling):
    no = 7
    speeds = [10, 10, 10, 10, 10]
    pos_name = ['_L2', '_L1', '_Mid', '_R1', '_R2']
    for dir in range(4):
        input("请将表面NO%d置于%d方向上，位置从%s开始：" % (no, dir+1, pos_name[0].split('_')[-1]))
        save_name = ['_NO' + str(no) + '_Dir' + str(dir + 1) + x for x in pos_name]
        sampling.goback_goback_continuous_batch(speeds, save_name=save_name, sample_block=True)


def get_lstm_all_surface_data(sampling):
    for i in range(8):
        input("请放置No%d表面: " % i)

        get_lstm_one_surface_data(sampling, "NO%dSurface" % i)
