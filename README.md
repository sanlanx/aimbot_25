A ambot demo of robomaster
abc
# 对配置文件等静态数据的修改在这标注一下
## ModelConfig.yaml
新建了这个文件  
添加了模型路径和分类文件路径：model_path, label_path

# model
## label.txt
由于模型的输出如下：  
0到8是四个关键点，顺序从左上角开始逆时针；9到13是颜色（红蓝灰紫），13到22是数字  
G（哨兵） 1（一号） 2（二号） 3（三号） 4（四号） 5（五号） O（前哨站） Bs（基地） Bb（基地大装甲）  
*每一个信息后面都有一个空位，可能是占位符？*  