import { createRoot } from 'react-dom/client'
import App from './App'
import PresentationApp from './PresentationApp'
import './styles.css'

const presentation = new URLSearchParams(window.location.search).has('presentation')

createRoot(document.getElementById('root')!).render(
  presentation ? <PresentationApp /> : <App />,
)
