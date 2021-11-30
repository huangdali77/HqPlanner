for (i = 0; i < r; i++) {
  j = 0;
  while (matrix[i][j] == 0) j++;
  if (matrix[i][j] != 0)
    for (k = 0; k < i; k++) {
      times = matrix[k][j] / matrix[i][j];
      for (l = 0; l < c; l++) matrix[k][l] -= times * matrix[i][l];
    }
}