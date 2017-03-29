int
slew (int current, float unitsPerSecond, float deltaTime) {
	return current + unitsPerSecond * deltaTime;
}