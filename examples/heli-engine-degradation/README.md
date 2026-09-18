# Helicopter scheduled engine degradation and loss

```bash
galata run examples/heli-engine-degradation/study.yaml --output-dir build/heli-engine
```

At fixed-step boundaries the study changes engine availability to 50% and then
to zero. The trajectory CSV is accompanied by
`engine-degradation.csv.events.txt`. Engine loss is deliberately not presented
as autorotation: that physics is outside this Level-1 model, and the run may
enter its declared vortex-ring envelope.
