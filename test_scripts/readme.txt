How to use:
1. Install tensorflow 1.9.0
2. Clone tensorflow models from https://github.com/tensorflow/models.git
3. Go to <somepath>/models/research
4. Execute: protoc object_detection/protos/*.proto --python_out=.
5. Execute: export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
6. Copy object_detection/utils folder and contents to <somepath>/CarND-Capstone/test_scripts
7. Go to <somepath>/CarND-Capstone/test_scripts
8. Execute: python simple_classification.py

Currently alex lechner dataset from https://github.com/alex-lechner/Traffic-Light-Classification is used for test and training