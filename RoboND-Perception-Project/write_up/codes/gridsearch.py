parameter_grid = [{'C':[0.1, 1,10,100,1000], 'kernel': ['linear']}, 
                  {'C': [0.1, 1, 10, 100, 1000], 'gamma': [0.01, 0.001, 0.0001], 'kernel': ['rbf']}, 
                  {'C': [0.1, 1, 10, 100, 1000], 'gamma': [0.1, 0.01, 0.001, 0.0001], 'kernel': ['sigmoid']}]
clf = GridSearchCV(estimator=svm.SVC(), param_grid=parameter_grid, n_jobs=-1)
....
....
clf.fit(X=X_train, y=y_train)
print('Best C:',clf.best_estimator_.C) 
print('Best Kernel:',clf.best_estimator_.kernel)
print('Best Gamma:',clf.best_estimator_.gamma)