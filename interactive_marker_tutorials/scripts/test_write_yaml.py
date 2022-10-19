import yaml 

with open('write_sample.yaml', 'w') as f:
    data = {
        'setting': {
            'language' : 'jp'
        }
    }
    list = {'list': [1, 2, 3]}
    num = [1, 2, 3]
    data['setting']['list'] = []
    for i in range(2):
        # data['setting']['list'] = [1, 2, 3]
        # data['setting']['list'] = []
        data['setting']['list'].append(num)
        # data['setting']['select'] = list
        
    yaml.dump(data, f)