from pytest import approx

from v2calculations import Csy

def test_csy_conversions():
  csy = Csy([ 653.0, 134.0, 126.5 ], [0, -90, 0])

  mat = csy.toMatrix4()
  newcsy = Csy.fromMatrix4(mat)

  assert newcsy.origin == approx(csy.origin)
  assert newcsy.euler == approx(csy.euler)

def test_csy_conversions2():
  csy = Csy([ 653.0, 134.0, 126.5 ], [34, 34, 155])

  mat = csy.toMatrix4()
  newcsy = Csy.fromMatrix4(mat)

  assert newcsy.origin == approx(csy.origin)
  assert newcsy.euler == approx(csy.euler)
