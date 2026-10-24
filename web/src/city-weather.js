export const CITY_WEATHER = Object.freeze({
  clear: { label:'CLEAR', extra:0, interval:Infinity },
  rain: { label:'RAIN', extra:2, interval:9 },
  snow: { label:'SNOW', extra:4, interval:6 },
});
export function weatherSupply(base, weather, strength, reserve) {
  const p=CITY_WEATHER[weather];
  if(!p || !Number.isFinite(strength) || strength<0 || strength>1) throw new Error('Invalid city weather');
  return Math.min(16,base+(reserve?Math.ceil(p.extra*strength):0));
}
