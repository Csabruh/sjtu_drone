import numpy as np
import pandas as pd
from sklearn.feature_selection import SelectKBest
from sklearn.feature_selection import f_classif
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report

#from micromlgen import port  C file-ként menti ki
import joblib
from collections import Counter

'''

    F - forward         w
    I - forward right   -
    R - right           d
    G - forward left    -
    L - left            a

'''

def most_frequent(List):
    occurence_count = Counter(List)
    return occurence_count.most_common(1)[0][0]

data = pd.read_csv('./drone_stuff/src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/ml_dataset/DataSet_1.txt', header=None)
data.rename(columns={data.columns[-1]: 'Label'}, inplace=True)
data = data[(data['Label'] != 'L') & (data['Label'] != 'R') & (data['Label'] != 'H') & (data['Label'] != 'J') & (data['Label'] != 's')]
data.reset_index(drop=True, inplace=True)
X = data.iloc[:, :-1]
y = data.iloc[:, -1]

label_encoder = LabelEncoder()
y = label_encoder.fit_transform(y)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

k = 80
k_best = SelectKBest(score_func=f_classif, k=k)
k_best.fit(X_train, y_train)

selected_feature_indices = k_best.get_support(indices=True)
print("selected_feature_indices: ", selected_feature_indices)


clf = RandomForestClassifier(max_depth=3, random_state=42)
clf.fit(X_train.iloc[:, selected_feature_indices], y_train)

y_pred = clf.predict(X_test.iloc[:, selected_feature_indices])

accuracy = accuracy_score(y_test, y_pred)
print(f'Accuracy: {accuracy}')

class_names = label_encoder.classes_
report = classification_report(y_test, y_pred, target_names=class_names, zero_division=0)
print('Classification Report:\n', report)

#Menti a file-t
joblib.dump(clf, "./drone_stuff/src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/ml_dataset/random_forest_1.joblib")

#Betölti a file-t
loaded_rf = joblib.load("./drone_stuff/src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/ml_dataset/random_forest_1.joblib") 
ad = loaded_rf.predict(X_test.iloc[:, selected_feature_indices])

print(most_frequent(ad))
print(most_frequent(y_pred))
print(X_test)
print(X_test.iloc[:, selected_feature_indices])

 

   


#raspy_code = open("raspy_random_forest10.c", mode="w+")
#raspy_code.write(port(clf))