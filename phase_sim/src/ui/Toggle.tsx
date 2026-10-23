import './toggle.css';

/** Two-way segmented toggle, e.g. "Shared dims | Built kart". */
export function Toggle<T extends string>({ options, value, onChange, 'aria-label': ariaLabel }: {
  options: Array<{ value: T; label: string }>;
  value: T;
  onChange: (next: T) => void;
  'aria-label': string;
}) {
  return (
    <div className="ui-toggle" role="group" aria-label={ariaLabel}>
      {options.map((opt) => (
        <button
          key={opt.value}
          type="button"
          className="ui-toggle-opt"
          aria-pressed={value === opt.value}
          onClick={() => onChange(opt.value)}
        >
          {opt.label}
        </button>
      ))}
    </div>
  );
}
